#pragma once
#include <vector>
#include <memory>
#include "Landmark.hpp"

namespace visual_slam_2d {

class World {
 public:
  World() {}
    void addLandMark(std::shared_ptr<Landmark> lm) {
        landmarks_.push_back(lm);
    }

 private:
  std::vector<std::shared_ptr<Landmark>> landmarks_;
};

}  // namespace visual_slam_2d