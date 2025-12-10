#include "PathManager.h"

namespace PathManager {
    void DirectDrive::Init(Vector2d Start,Vector2d Goal){
        this->Start = Start;
        this->Goal = Goal;
        UpdateObstacles();
        std::cout << "[PathManager::Dstar::Init] Init Started" << std::endl;

  int width = cfg::Dimensions::fieldWidth / cfg::SystemConfig::Accuracy;
  int height = cfg::Dimensions::fieldHeight / cfg::SystemConfig::Accuracy;
  std::cout << "[PathManager::Dstar::Init] Actual Goal: " << Goal.x() << " " << Goal.y()
            << std::endl;
  for (int h = 0; h < height; h++) {
    cfg::SystemConfig::grid.push_back({});
    for (int w = 0; w < width; w++) {
      cfg::SystemConfig::grid[h].push_back(Vector3d);
    }
  }

  std::cout << "[PathManager::Dstar::Init] Grid created: " << width << "x" << height << std::endl;

    }
    
}