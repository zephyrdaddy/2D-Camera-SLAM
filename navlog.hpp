#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem> 

#include <Eigen/Core>
#include <sophus/se2.hpp>

#include "pose2d.hpp"
#include "robot.hpp"
#include "world.hpp"
#include "camera_model.hpp"
#include "camera_sensor.hpp"
#include "landmark.hpp"

namespace visual_slam_2d {

//------------------------------------------------------------------------------
// Versioning
//------------------------------------------------------------------------------

constexpr std::uint16_t kNavlogVersion = 1;

//------------------------------------------------------------------------------
// On-disk header
//------------------------------------------------------------------------------

struct NavlogHeader {
    char magic[4];        // "NAVL"
    std::uint16_t version;
    std::uint16_t flags;  // reserved for future use (endianness, etc.)
};

//------------------------------------------------------------------------------
// Static metadata POD structs
//------------------------------------------------------------------------------

struct NavlogCameraInfo {
    float focal_length;
    std::int32_t width;
    float extrinsic_x;
    float extrinsic_y;
    float extrinsic_theta;
};

struct NavlogLandmarkInfo {
    std::uint32_t id;
    float x;
    float y;
};

//------------------------------------------------------------------------------
// Event types and POD payloads
//------------------------------------------------------------------------------

enum class NavlogEventType : std::uint8_t {
    kAction      = 1,
    kObservation = 2,
    kEstimate    = 3,
};

// Per-event header
struct NavlogEventHeader {
    double timestamp;        // seconds or step index
    std::uint8_t event_type; // NavlogEventType
    std::uint8_t reserved[7];
};

// Action event (example: velocity command)
struct NavlogActionEvent {
    float vx;    // forward velocity in robot frame
    float vy;    // lateral velocity (if unused, set 0)
    float omega; // angular velocity
};

// Observation event: 1D camera measurement of a landmark
struct NavlogObservationEvent {
    std::uint32_t landmark_id;
    std::uint32_t camera_index;
    float u;           // projected 1D coordinate
    std::uint8_t valid; // 1 = valid, 0 = invalid
    std::uint8_t reserved[3];
};

// Estimate event: one pose estimate at given time
struct NavlogEstimateEvent {
    float x;
    float y;
    float theta;
    // role_flags: bit 0=prior, bit 1=correction, bit 2=final
    std::uint8_t role_flags;
    std::uint8_t reserved[3];
};

//------------------------------------------------------------------------------
// Helpers to extract POD metadata from your objects
//------------------------------------------------------------------------------

// NOTE: you need to add these simple getters if they don't exist yet:
//
// camera_model.hpp:
//   float width() const { return width_; }
//
// camera_sensor.hpp:
//   const Pose2D& extrinsic() const { return cam_pose_rel_; }
//   const CameraModel& model() const { return cam_model_; }
//
// landmark.hpp:
//   std::uint32_t id() const { return id_; }
//   const Eigen::Vector2f& position() const { return pos_; }
//
// world.hpp:
//   const std::vector<std::shared_ptr<Landmark>>& landmarks() const { return landmarks_; }

// Extract camera info from Robot
inline std::vector<NavlogCameraInfo> extractCameraInfos(const Robot& robot) {
    std::vector<NavlogCameraInfo> out;
    const auto& cams = robot.cameras();
    out.reserve(cams.size());

    for (const auto& cam_ptr : cams) {
        const auto& cam = *cam_ptr;
        NavlogCameraInfo info;
        info.focal_length    = cam.model().focal_length();
        info.width           = static_cast<std::int32_t>(cam.model().width());
        info.extrinsic_x     = cam.extrinsic().translation().x();
        info.extrinsic_y     = cam.extrinsic().translation().y();
        info.extrinsic_theta = cam.extrinsic().so2().log();
        out.push_back(info);
    }
    return out;
}

// Extract landmark info from World
inline std::vector<NavlogLandmarkInfo> extractLandmarkInfos(const World& world) {
    std::vector<NavlogLandmarkInfo> out;
    const auto& lms = world.landmarks();
    out.reserve(lms.size());

    for (const auto& lm_ptr : lms) {
        const auto& lm = *lm_ptr;
        NavlogLandmarkInfo info;
        info.id = lm.id();
        info.x  = lm.position().x();
        info.y  = lm.position().y();
        out.push_back(info);
    }
    return out;
}

//------------------------------------------------------------------------------
// NavlogWriter: main API to create a .navlog file
//------------------------------------------------------------------------------

class NavlogWriter {
public:
    explicit NavlogWriter(const std::string& path)
    {

        namespace fs = std::filesystem;

        fs::path p(path);
        if (p.has_parent_path()) {
            std::error_code ec;
            fs::create_directories(p.parent_path(), ec);  // ignore error if already exists
            // You could optionally check ec and throw/log if needed.
        }

        ofs_.open(path, std::ios::binary);
        if (!ofs_) {
            throw std::runtime_error("Failed to open navlog file for writing: " + path);
        }

        NavlogHeader header;
        header.magic[0] = 'N';
        header.magic[1] = 'A';
        header.magic[2] = 'V';
        header.magic[3] = 'L';
        header.version  = kNavlogVersion;
        header.flags    = 0;

        writeRaw(header);
    }

    bool good() const { return ofs_.good(); }

    // Write static metadata once at the start of a scenario.
    void writeMetadata(const Pose2D& world_origin,
                       const std::vector<NavlogCameraInfo>& cameras,
                       const std::vector<NavlogLandmarkInfo>& landmarks)
    {
        writePose(world_origin);

        std::uint32_t num_cams =
            static_cast<std::uint32_t>(cameras.size());
        writeRaw(num_cams);
        for (const auto& cam : cameras) {
            writeRaw(cam);
        }

        std::uint32_t num_landmarks =
            static_cast<std::uint32_t>(landmarks.size());
        writeRaw(num_landmarks);
        for (const auto& lm : landmarks) {
            writeRaw(lm);
        }
    }

    // Convenience overload using Robot + World directly.
    void writeMetadata(const Pose2D& world_origin,
                       const Robot& robot,
                       const World& world)
    {
        auto cams = extractCameraInfos(robot);
        auto lms  = extractLandmarkInfos(world);
        writeMetadata(world_origin, cams, lms);
    }

    // Log an action (optional for now).
    void logAction(double t, const NavlogActionEvent& action) {
        NavlogEventHeader eh{};
        eh.timestamp  = t;
        eh.event_type = static_cast<std::uint8_t>(NavlogEventType::kAction);
        writeRaw(eh);
        writeRaw(action);
    }

    // Log an observation.
    void logObservation(double t, const NavlogObservationEvent& obs) {
        NavlogEventHeader eh{};
        eh.timestamp  = t;
        eh.event_type = static_cast<std::uint8_t>(NavlogEventType::kObservation);
        writeRaw(eh);
        writeRaw(obs);
    }

    // Log a pose estimate (prior/correction/final).
    void logEstimate(double t, const NavlogEstimateEvent& est) {
        NavlogEventHeader eh{};
        eh.timestamp  = t;
        eh.event_type = static_cast<std::uint8_t>(NavlogEventType::kEstimate);
        writeRaw(eh);
        writeRaw(est);
    }

    void flush() { ofs_.flush(); }

private:
    std::ofstream ofs_;

    template <typename T>
    void writeRaw(const T& v) {
        ofs_.write(reinterpret_cast<const char*>(&v), sizeof(T));
    }

    void writePose(const Pose2D& p) {
        const Eigen::Vector2f t = p.translation();
        const float theta = p.so2().log();
        writeRaw(t.x());
        writeRaw(t.y());
        writeRaw(theta);
    }
};

} // namespace visual_slam_2d
