#include "PathManager.h"

namespace PathManager {
void DirectDrive::Init(Eigen::Vector2d Start, Eigen::Vector2d Goal) {
  this->Start = Start;
  this->Goal = Goal;
}
float DirectDrive::Euclidean(Eigen::Vector2d PointA, Eigen::Vector2d PointB) {
  return sqrt(pow(PointA.x() - PointB.x(), 2) + pow(PointA.y() - PointB.y(), 2));
}

std::vector<Eigen::Vector2d> DirectDrive::getObstacles(float Grad, float Beta) {
  std::vector<Eigen::Vector2d> Obstacles;
  for (Eigen::Vector3d Pos : cfg::SystemConfig::teamOnePlayerPos) {
    if (Pos.y() == Start.y() && Pos.x() == Start.x()) continue;
    if (!(std::min(Start.y(), Goal.y()) <= Pos.y() && Pos.y() <= std::max(Start.y(), Goal.y()) &&
          std::min(Start.x(), Goal.x()) <= Pos.x() && Pos.x() < std::max(Start.x(), Goal.x())))
      continue;
    float xh = Pos.x();
    float yk = Pos.y();
    if (xh == 0 && yk == 0) {
      continue;
    }
    if (pow(Grad * xh - yk + Beta, 2) <=
        (cfg::SystemConfig::robotRadius * cfg::SystemConfig::robotRadius) * (1 + (Grad * Grad)))
      Obstacles.push_back(Pos.head<2>());
  }
  return Obstacles;
};
bool compare(std::pair<float, int> a, std::pair<float, int> b) { return a.first < b.first; }
void DirectDrive::CreatePath(Eigen::Vector2d Start, Eigen::Vector2d End) {
  std::vector<Eigen::Vector2d> Path;
  if (End != this->Goal) cfg::SystemConfig::teamOnePath[this->RobotInd - 1].clear();
  this->Goal = End;
  float run = Goal.x() - Start.x();
  float rise = Goal.y() - Start.y();
  float Grad = rise / run;
  float Beta = Start.y() - (Start.x() * Grad);
  float radius = cfg::SystemConfig::robotRadius;
  std::vector<Eigen::Vector2d> Obstacles = getObstacles(Grad, Beta);
  std::cout << "[PathManager::DirectDrive::CreatePath] Obstacles detected: " << Obstacles.size()
            << std::endl;
  while (Obstacles.size()) {
    std::vector<std::pair<float, int>> Distances;
    for (int i = 0; i < Obstacles.size(); i++) {
      Distances.push_back(std::make_pair((Start - Obstacles[i]).norm(), i));
    }
    std::sort(Distances.begin(), Distances.end(), compare);

    int RobotInd = Distances[0].second;
    Eigen::Vector2d CurrObs = Obstacles[RobotInd];
    Distances.clear();
    Start = Obstacles[RobotInd];
    while (std::find(Obstacles.begin(), Obstacles.end(), CurrObs) != Obstacles.end()) {
      Start.y() += Epsilon;
      if (Start.y() > cfg::Dimensions::fieldHeight) {
        std::cout
            << "[PathManager::DirectDrive::CreatePath] Height is out of bounds, Breaking loop."
            << std::endl;
        return;
      }
      run = Goal.x() - Start.x();
      rise = Goal.y() - Start.y();
      Grad = rise / run;
      Beta = Start.y() - (Start.x() * Grad);
      Obstacles = getObstacles(Grad, Beta);
      std::cout << "[PathManager::DirectDrive::CreatePath] Obstacles detected: "
                << Obstacles.size() << std::endl;
      std::cout << "[PathManager::DirectDrive::CreatePath] Obstacles: ";
      for (auto i : Obstacles) {
        std::cout << "(" << i.x() << " , " << i.y() << ") " << std::endl;
      }
      std::cout << "[PathManager::DirectDrive::CreatePath] New Start: " << Start.x() << " "
                << Start.y() << std::endl;
      ;
    }
    Path.push_back(Start);
  }
  Path.push_back(Goal);
  std::cout << "[PathManager::DirectDrive::CreatePath] Path: ";
  for (auto i : Path) {
    cfg::SystemConfig::teamOnePath[this->RobotInd - 1].push_back(Eigen::Vector3d(i.x(), i.y(), 0));
    std::cout << "(" << i.x() << " , " << i.y() << ") " << std::endl;
  }
  std::cout << std::endl;
};
}  // namespace PathManager