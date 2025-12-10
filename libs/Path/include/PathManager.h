#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H
#include "systemConfig.h"
#include "dimensions.h"
#include "Eigen/Dense"
#include "queue"
#define Vector2d Eigen::Vector2d
#include <iostream>
namespace PathManager {

class DirectDrive {
 private:
    Vector2d Goal;
    Vector2d Start;
 public:
    void Init(Vector2d Start, Vector2d Goal);
    void FindPath();
    void UpdateObstacles();
};
}  // namespace PathManager

#endif