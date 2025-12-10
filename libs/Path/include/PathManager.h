#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H
#include "systemConfig.h"
#include "dimensions.h"
#include "Eigen/Dense"
#include "queue"

#include <iostream>
namespace PathManager {
struct NodeCompare {
  bool operator()(cfg::Node a, cfg::Node b) {
    if (a.Key.x() != b.Key.x()) return a.Key.x() > b.Key.y();
    return a.Key.y() > b.Key.y();
  }
};
class Dstar {
 private:
  cfg::Node Goal;
  float km = 0.0f;
  cfg::Node prevStart;
  cfg::Node prevGoal;
  std::vector<std::pair<int, int>> indices;
  std::vector<cfg::Node> getNeighbors(cfg::Node u);

  Eigen::Vector2d calculateKey(cfg::Node u);
  std::priority_queue<cfg::Node, std::vector<cfg::Node>, NodeCompare> openQueue;
  std::vector<PathManager::Dstar> PathManagers;
  cfg::Node transform(cfg::Node node_);
  void updateObstacles();
  void tunePath();
  void MakePath();
  void updateVertex(cfg::Node u);
  void computeShortestPath();

  bool constrain(int first, int second);
  float heuristic(cfg::Node a, cfg::Node b);
  void extractPath();

 public:
  int RobotInd;
  cfg::Node Start;
  void Init(Eigen::Vector2d Start, Eigen::Vector2d Goal);
  void update(Eigen::Vector2d Start, Eigen::Vector2d Goal);
};
}  // namespace PathManager

#endif