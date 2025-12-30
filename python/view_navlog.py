import argparse
import struct
from dataclasses import dataclass
from typing import BinaryIO, List

import numpy as np
import rerun as rr  # pip install rerun-sdk  (package name may be `rerun-sdk` or similar) [web:28]


NAVLOG_MAGIC = b"NAVL"
NAVLOG_VERSION = 1


@dataclass
class CameraInfo:
    focal_length: float
    width: int
    extrinsic_x: float
    extrinsic_y: float
    extrinsic_theta: float


@dataclass
class LandmarkInfo:
    id: int
    x: float
    y: float


@dataclass
class ActionEvent:
    t: float
    vx: float
    vy: float
    omega: float


@dataclass
class ObservationEvent:
    t: float
    landmark_id: int
    camera_index: int
    u: float
    valid: bool


@dataclass
class EstimateEvent:
    t: float
    x: float
    y: float
    theta: float
    role_flags: int  # bit 0=prior, bit 1=correction, bit 2=final


@dataclass
class Navlog:
    world_origin: tuple  # (x, y, theta)
    cameras: List[CameraInfo]
    landmarks: List[LandmarkInfo]
    actions: List[ActionEvent]
    observations: List[ObservationEvent]
    estimates: List[EstimateEvent]


def read_exact(f: BinaryIO, n: int) -> bytes:
    data = f.read(n)
    if len(data) != n:
        raise EOFError("Unexpected end of file")
    return data


def parse_navlog(path: str) -> Navlog:
    with open(path, "rb") as f:
        # Header: magic[4], version(uint16), flags(uint16)
        header = read_exact(f, 8)
        magic, version, flags = struct.unpack("<4sHH", header)
        if magic != NAVLOG_MAGIC:
            raise ValueError(f"Bad magic: {magic!r}")
        if version != NAVLOG_VERSION:
            raise ValueError(f"Unsupported version: {version}")

        # World origin pose: x(float), y(float), theta(float)
        world_origin = struct.unpack("<fff", read_exact(f, 12))

        # Cameras
        (num_cams,) = struct.unpack("<I", read_exact(f, 4))
        cameras: List[CameraInfo] = []
        for _ in range(num_cams):
            # NavlogCameraInfo: float, int32, float, float, float
            focal_length, width, ex, ey, eth = struct.unpack("<fifff", read_exact(f, 4 + 4 + 4 + 4 + 4))
            cameras.append(CameraInfo(focal_length, width, ex, ey, eth))

        # Landmarks
        (num_lms,) = struct.unpack("<I", read_exact(f, 4))
        landmarks: List[LandmarkInfo] = []
        for _ in range(num_lms):
            lm_id, x, y = struct.unpack("<Iff", read_exact(f, 4 + 4 + 4))
            landmarks.append(LandmarkInfo(lm_id, x, y))

        actions: List[ActionEvent] = []
        observations: List[ObservationEvent] = []
        estimates: List[EstimateEvent] = []

        # Read events until EOF
        while True:
            try:
                # NavlogEventHeader: double timestamp, uint8 type, 7 bytes reserved
                hdr_bytes = read_exact(f, 8 + 1 + 7)
            except EOFError:
                break

            t, etype = struct.unpack("<dB", hdr_bytes[:9])

            if etype == 1:  # kAction
                vx, vy, omega = struct.unpack("<fff", read_exact(f, 12))
                actions.append(ActionEvent(t, vx, vy, omega))

            elif etype == 2:  # kObservation
                lm_id, cam_idx, u, valid, _r1, _r2, _r3 = struct.unpack("<IIfBBBB", read_exact(f, 4 + 4 + 4 + 1 + 3))
                observations.append(ObservationEvent(t, lm_id, cam_idx, u, bool(valid)))

            elif etype == 3:  # kEstimate
                x, y, theta, role_flags, _r1, _r2, _r3 = struct.unpack("<fffBBBB", read_exact(f, 4 + 4 + 4 + 1 + 3))
                estimates.append(EstimateEvent(t, x, y, theta, role_flags))

            else:
                raise ValueError(f"Unknown event type: {etype}")

    return Navlog(
        world_origin=world_origin,
        cameras=cameras,
        landmarks=landmarks,
        actions=actions,
        observations=observations,
        estimates=estimates,
    )


def visualize_navlog(navlog: Navlog, recording_name: str = "navlog") -> None:
    # Initialize Rerun and spawn a viewer window. [web:28]
    rr.init(recording_name, spawn=True)

    # Log landmarks as 3D points with z=0. [web:36][web:33]
    if navlog.landmarks:
        positions = np.array([[lm.x, lm.y, 0.0] for lm in navlog.landmarks], dtype=np.float32)
        labels = [str(lm.id) for lm in navlog.landmarks]
        rr.log(
            "world/landmarks",
            rr.Points3D(positions, labels=labels),
        )

    # Extract prior and final estimates (if present).
    priors = [e for e in navlog.estimates if e.role_flags & 0b00000001]
    finals = [e for e in navlog.estimates if e.role_flags & 0b00000100]

    # Log robot poses over time on a "time" timeline. [web:28][web:35]
    for e in navlog.estimates:
        rr.set_time_seconds("time", e.t)
        # For now, treat orientation as an arrow in XY plane only: use points for pose.
        rr.log(
            "robot/pose_est",
            rr.Points3D(np.array([[e.x, e.y, 0.0]], dtype=np.float32)),
        )

    # Optionally, connect final trajectory as a line strip.
    if navlog.estimates:
        ests_sorted = sorted(navlog.estimates, key=lambda e: e.t)
        traj = np.array([[e.x, e.y, 0.0] for e in ests_sorted], dtype=np.float32)
        rr.log(
            "robot/trajectory_est",
            rr.LineStrips3D([traj]),
        )

    # Visualize observation rays at their timestamps.
    # For now we draw from robot pose at that time to the landmark position (z=0). [web:30]
    lm_by_id = {lm.id: lm for lm in navlog.landmarks}

    for obs in navlog.observations:
        if not obs.valid:
            continue
        lm = lm_by_id.get(obs.landmark_id)
        if lm is None:
            continue

        # Find closest estimate in time (simple nearest-neighbor).
        if not navlog.estimates:
            continue
        est = min(navlog.estimates, key=lambda e: abs(e.t - obs.t))

        rr.set_time_seconds("time", obs.t)

        start = np.array([[est.x, est.y, 0.0]], dtype=np.float32)
        end = np.array([[lm.x, lm.y, 0.0]], dtype=np.float32)
        line = np.vstack([start, end])

        rr.log(
            "debug/obs_rays",
            rr.LineStrips3D([line]),
        )

    # Viewer will stay open; script exits when window is closed.


def main():
    parser = argparse.ArgumentParser(description="View navlog with Rerun")
    parser.add_argument("navlog", help="Path to .navlog file")
    args = parser.parse_args()

    navlog = parse_navlog(args.navlog)
    visualize_navlog(navlog)


if __name__ == "__main__":
    main()
