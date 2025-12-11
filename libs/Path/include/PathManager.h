#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H
#include "systemConfig.h"
#include "dimensions.h"
#include "Eigen/Dense"
#include "queue"

// #define Vector2d Eigen::Eigen::Vector2d
#include <iostream>
namespace PathManager {

class DirectDrive {
 private:
  Eigen::Vector2d Goal;
  float Epsilon = 0.1;
  float Euclidean(Eigen::Vector2d PointA, Eigen::Vector2d PointB);
  std::vector<Eigen::Vector2d> getObstacles(float Grad,float Beta);

 public:
  Eigen::Vector2d Start;
  void Init(Eigen::Vector2d Start, Eigen::Vector2d Goal);
  void CreatePath(Eigen::Vector2d Start, Eigen::Vector2d End);
  int RobotInd;
};
}  // namespace PathManager

#endif