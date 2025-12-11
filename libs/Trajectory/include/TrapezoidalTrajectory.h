#ifndef TRAPEZOIDAL_TRAJECTORY_H
#define TRAPEZOIDAL_TRAJECTORY_H
#include <iostream>
#include <cmath>
#include "systemConfig.h"
#include "PathManager.h"
#include <vector>
#define dt 1 / cfg::SystemConfig::frameRate

namespace Traj {
class Trapezoidal_Traj {
 private:
  std::vector<bool> init;

 public:
 PathManager::DirectDrive Path;
  Trapezoidal_Traj();
  void SetVelocityFromTraj(int index);
};
}  // namespace Traj

#endif
