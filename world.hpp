#pragma once
#include <vector>
#include <memory>
#include "landmark.hpp"

namespace visual_slam_2d {

class World {
 public:
  World() {}

  void addLandMark(std::shared_ptr<Landmark> lm) {
      landmarks_.push_back(lm);
  }

  // Read-only access to landmarks (for logging / visualization, etc.).
  const std::vector<std::shared_ptr<Landmark>>& landmarks() const {
      return landmarks_;
  }

  // Optional: number of landmarks.
  std::size_t numLandmarks() const {
      return landmarks_.size();
  }

 private:
  std::vector<std::shared_ptr<Landmark>> landmarks_;
};

}  // namespace visual_slam_2d
