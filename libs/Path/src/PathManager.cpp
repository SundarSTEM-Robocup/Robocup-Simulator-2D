#include "PathManager.h"

namespace PathManager {

void Dstar::Init(Eigen::Vector2d Start, Eigen::Vector2d Goal) {
  std::cout << "[PathManager::Dstar::Init] Init Started" << std::endl;

  int width = cfg::Dimensions::fieldWidth / cfg::SystemConfig::Accuracy;
  int height = cfg::Dimensions::fieldHeight / cfg::SystemConfig::Accuracy;
  std::cout << "[PathManager::Dstar::Init] Actual Goal: " << Goal.x() << " " << Goal.y()
            << std::endl;
  for (int h = 0; h < height; h++) {
    cfg::SystemConfig::grid.push_back({});
    for (int w = 0; w < width; w++) {
      cfg::Node node_;
      node_.Pos = Eigen::Vector2d(w, h);
      node_.g = INFINITY;
      node_.rhs = INFINITY;
      node_.Key = Eigen::Vector2d(INFINITY, INFINITY);
      cfg::SystemConfig::grid[h].push_back(node_);
    }
  }

  std::cout << "[PathManager::Dstar::Init] Grid created: " << width << "x" << height << std::endl;

  this->Start.Pos = Start.head<2>();
  this->Goal.Pos = Goal.head<2>();
  this->Start = transform(this->Start);
  this->Goal = transform(this->Goal);

  std::cout << "[PathManager::Dstar::Init] Start: (" << this->Start.Pos.x() << ","
            << this->Start.Pos.y() << ")" << std::endl;
  std::cout << "[PathManager::Dstar::Init] Goal: (" << this->Goal.Pos.x() << ","
            << this->Goal.Pos.y() << ")" << std::endl;

  int gx = this->Goal.Pos.x();
  int gy = this->Goal.Pos.y();
  cfg::SystemConfig::grid[gy][gx].rhs = 0.0f;
  cfg::SystemConfig::grid[gy][gx].Key = calculateKey(cfg::SystemConfig::grid[gy][gx]);
  openQueue.push(cfg::SystemConfig::grid[gy][gx]);

  km = 0.0f;

  std::cout << "[PathManager::Dstar::Init] Computing initial path" << std::endl;
  computeShortestPath();

  std::cout << "[PathManager::Dstar::Init] Init Finished" << std::endl;
}

cfg::Node Dstar::transform(cfg::Node node_) {
  cfg::Node Transformed;
  Transformed.g = node_.g;
  Transformed.Key = node_.Key;
  Transformed.obstacle = node_.obstacle;
  Transformed.rhs = node_.rhs;
  Transformed.Pos = Eigen::Vector2d(
      (int)((node_.Pos.x() + cfg::Dimensions::fieldWidth / 2.0) / cfg::SystemConfig::Accuracy),
      (int)((node_.Pos.y() + cfg::Dimensions::fieldHeight / 2.0) / cfg::SystemConfig::Accuracy));
  return Transformed;
}

std::vector<cfg::Node> Dstar::getNeighbors(cfg::Node u) {
  std::vector<cfg::Node> neighbors;

  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      if (dx == 0 && dy == 0) continue;

      int nx = u.Pos.x() + dx;
      int ny = u.Pos.y() + dy;

      if (constrain(nx, ny) && !cfg::SystemConfig::grid[ny][nx].obstacle) {
        neighbors.push_back(cfg::SystemConfig::grid[ny][nx]);
      }
    }
  }

  return neighbors;
}

bool Dstar::constrain(int x, int y) {
  return x >= 0 && y >= 0 && y < cfg::SystemConfig::grid.size() &&
         x < cfg::SystemConfig::grid[0].size();
}

float Dstar::heuristic(cfg::Node a, cfg::Node b) {
  float dx = a.Pos.x() - b.Pos.x();
  float dy = a.Pos.y() - b.Pos.y();
  return sqrt(dx * dx + dy * dy);
}

Eigen::Vector2d Dstar::calculateKey(cfg::Node u) {
  float minVal = std::min(u.g, u.rhs);
  int sx = Start.Pos.x();
  int sy = Start.Pos.y();
  cfg::Node startNode = cfg::SystemConfig::grid[sy][sx];

  float k1 = minVal + heuristic(startNode, u) + km;
  float k2 = minVal;

  return Eigen::Vector2d(k1, k2);
}

void Dstar::updateVertex(cfg::Node u) {
  int ux = u.Pos.x();
  int uy = u.Pos.y();

  std::priority_queue<cfg::Node, std::vector<cfg::Node>, NodeCompare> tempQueue;
  while (!openQueue.empty()) {
    cfg::Node top = openQueue.top();
    openQueue.pop();
    if (top.Pos.x() != ux || top.Pos.y() != uy) {
      tempQueue.push(top);
    }
  }
  openQueue = tempQueue;

  int gx = Goal.Pos.x();
  int gy = Goal.Pos.y();
  if (ux != gx || uy != gy) {
    std::vector<cfg::Node> neighbors = getNeighbors(u);
    float minRhs = INFINITY;

    for (cfg::Node s : neighbors) {
      int sx = s.Pos.x();
      int sy = s.Pos.y();
      float cost = heuristic(u, s);
      float newRhs = cfg::SystemConfig::grid[sy][sx].g + cost;
      if (newRhs < minRhs) {
        minRhs = newRhs;
      }
    }

    cfg::SystemConfig::grid[uy][ux].rhs = minRhs;
  }

  if (abs(cfg::SystemConfig::grid[uy][ux].g - cfg::SystemConfig::grid[uy][ux].rhs) > 0.00001) {
    cfg::SystemConfig::grid[uy][ux].Key = calculateKey(cfg::SystemConfig::grid[uy][ux]);
    openQueue.push(cfg::SystemConfig::grid[uy][ux]);
  }
}

void Dstar::updateObstacles() {
  std::cout << "[PathManager::Dstar::updateObstacles] Updating obstacles" << std::endl;

  int limitx = cfg::SystemConfig::robotRadius / cfg::SystemConfig::Accuracy;
  int limity = cfg::SystemConfig::robotRadius / cfg::SystemConfig::Accuracy;

  for (std::pair<int, int> ind : indices) {
    if (constrain(ind.first, ind.second)) {
      cfg::SystemConfig::grid[ind.second][ind.first].obstacle = false;
    }
  }
  indices.clear();

  for (Eigen::Vector3d Pos : cfg::SystemConfig::teamOnePlayerPos) {
    cfg::Node Curr;
    Curr.Pos = Pos.head<2>();
    Curr = transform(Curr);

    int cx = Curr.Pos.x();
    int cy = Curr.Pos.y();

    for (int dy = -limity; dy <= limity; dy++) {
      for (int dx = -limitx; dx <= limitx; dx++) {
        int nx = cx + dx;
        int ny = cy + dy;
        if ((dx * dx + dy * dy) <= (limitx * limitx) && constrain(nx, ny)) {
          indices.push_back(std::make_pair(nx, ny));
          cfg::SystemConfig::grid[ny][nx].obstacle = true;
        }
      }
    }
  }

  for (std::pair<int, int> ind : indices) {
    int x = ind.first;
    int y = ind.second;
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        int nx = x + dx;
        int ny = y + dy;
        if (constrain(nx, ny)) {
          updateVertex(cfg::SystemConfig::grid[ny][nx]);
        }
      }
    }
  }

  std::cout << "[PathManager::Dstar::updateObstacles] Updated " << indices.size()
            << " obstacle cells" << std::endl;
}

void Dstar::computeShortestPath() {
  std::cout << "[PathManager::Dstar::computeShortestPath] Starting computation" << std::endl;

  int sx = Start.Pos.x();
  int sy = Start.Pos.y();
  int iterations = 0;
  int maxSteps = 80000;

  if (!constrain(sx, sy)) {
    std::cout << "[PathManager::Dstar::computeShortestPath] ERROR: Start out of bounds"
              << std::endl;
    return;
  }

  while (!openQueue.empty()) {
    cfg::Node startNode = cfg::SystemConfig::grid[sy][sx];
    cfg::Node topNode = openQueue.top();

    Eigen::Vector2d startKey = calculateKey(startNode);
    bool startInconsistent = abs(startNode.g - startNode.rhs) > 0.00001;

    bool topLessThanStart = (topNode.Key.x() < startKey.x() - 0.00001) ||
                            (abs(topNode.Key.x() - startKey.x()) < 0.00001 &&
                             topNode.Key.y() < startKey.y() - 0.00001);

    if (!(topLessThanStart || startInconsistent)) {
      std::cout << "[PathManager::Dstar::computeShortestPath] Terminating" << std::endl;
      break;
    }

    if (iterations++ > maxSteps) {
      std::cout << "[PathManager::Dstar::computeShortestPath] Max steps reached" << std::endl;
      break;
    }

    int ux = topNode.Pos.x();
    int uy = topNode.Pos.y();

    if (!constrain(ux, uy)) {
      openQueue.pop();
      continue;
    }

    cfg::Node u = cfg::SystemConfig::grid[uy][ux];
    Eigen::Vector2d kOld = topNode.Key;
    Eigen::Vector2d kNew = calculateKey(u);

    openQueue.pop();

    bool kOldLessThanKNew = (kOld.x() < kNew.x() - 0.00001) ||
                            (abs(kOld.x() - kNew.x()) < 0.00001 && kOld.y() < kNew.y() - 0.00001);

    if (kOldLessThanKNew) {
      cfg::SystemConfig::grid[uy][ux].Key = kNew;
      openQueue.push(cfg::SystemConfig::grid[uy][ux]);
    } else if (u.g > u.rhs + 0.00001) {
      cfg::SystemConfig::grid[uy][ux].g = cfg::SystemConfig::grid[uy][ux].rhs;

      std::vector<cfg::Node> neighbors = getNeighbors(u);
      for (cfg::Node s : neighbors) {
        updateVertex(s);
      }
    } else {
      cfg::SystemConfig::grid[uy][ux].g = INFINITY;

      std::vector<cfg::Node> neighbors = getNeighbors(u);
      for (cfg::Node s : neighbors) {
        updateVertex(s);
      }
      updateVertex(u);
    }

    if (iterations % 1000 == 0) {
      std::cout << "[PathManager::Dstar::computeShortestPath] Iteration " << iterations
                << std::endl;
    }
  }

  cfg::Node finalStart = cfg::SystemConfig::grid[sy][sx];
  std::cout << "[PathManager::Dstar::computeShortestPath] Finished after " << iterations
            << " iterations" << std::endl;
  std::cout << "[PathManager::Dstar::computeShortestPath] Start g=" << finalStart.g
            << " rhs=" << finalStart.rhs << std::endl;
}

void Dstar::MakePath() {
  std::cout << "[PathManager::Dstar::MakePath] Computing initial path" << std::endl;
  computeShortestPath();
}

void Dstar::tunePath() {
  std::cout << "[PathManager::Dstar::tunePath] Recomputing path" << std::endl;
  computeShortestPath();
}

void Dstar::extractPath() {
  std::cout << "[PathManager::Dstar::extractPath] Extracting path" << std::endl;

  int sx = Start.Pos.x();
  int sy = Start.Pos.y();
  int gx = Goal.Pos.x();
  int gy = Goal.Pos.y();

  if (!constrain(sx, sy) || !constrain(gx, gy)) {
    std::cout << "[PathManager::Dstar::extractPath] ERROR: Start or Goal out of bounds"
              << std::endl;
    return;
  }

  if (cfg::SystemConfig::grid[sy][sx].g >= INFINITY - 1) {
    std::cout << "[PathManager::Dstar::extractPath] ERROR: No path exists" << std::endl;
    return;
  }

  std::vector<Eigen::Vector2d> path;
  std::vector<std::vector<bool>> visited(
      cfg::SystemConfig::grid.size(), std::vector<bool>(cfg::SystemConfig::grid[0].size(), false));
  int cx = sx;
  int cy = sy;

  int maxSteps = 10000;
  int steps = 0;

  while (cx != gx || cy != gy) {
    if (visited[cy][cx]) {
      std::cout << "[PathManager::Dstar::extractPath] ERROR: Cycle detected at (" << cx << ","
                << cy << ")" << std::endl;
      break;
    }

    visited[cy][cx] = true;
    path.push_back(Eigen::Vector2d(cx, cy));

    cfg::Node curr = cfg::SystemConfig::grid[cy][cx];
    std::vector<cfg::Node> neighbors = getNeighbors(curr);

    if (neighbors.empty()) {
      std::cout << "[PathManager::Dstar::extractPath] ERROR: No neighbors" << std::endl;
      break;
    }

    float minCost = INFINITY;
    float minTieBreaker = INFINITY;
    int nextX = cx;
    int nextY = cy;

    for (cfg::Node n : neighbors) {
      int nx = n.Pos.x();
      int ny = n.Pos.y();

      if (visited[ny][nx]) continue;

      float cost = cfg::SystemConfig::grid[ny][nx].g + heuristic(curr, n);
      float tieBreaker =
          heuristic(cfg::SystemConfig::grid[ny][nx], cfg::SystemConfig::grid[gy][gx]);

      if (cost < minCost - 0.00001 ||
          (abs(cost - minCost) < 0.00001 && tieBreaker < minTieBreaker)) {
        minCost = cost;
        minTieBreaker = tieBreaker;
        nextX = nx;
        nextY = ny;
      }
    }

    if (nextX == cx && nextY == cy) {
      std::cout << "[PathManager::Dstar::extractPath] ERROR: Stuck at (" << cx << "," << cy << ")"
                << std::endl;
      break;
    }

    cx = nextX;
    cy = nextY;
    steps++;

    if (steps >= maxSteps) {
      std::cout << "[PathManager::Dstar::extractPath] ERROR: Max steps reached" << std::endl;
      break;
    }
  }

  path.push_back(Eigen::Vector2d(gx, gy));
  path.erase(path.begin());
  std::cout << "[PathManager::Dstar::extractPath] Path length: " << path.size() << std::endl;
  std::cout << "[PathManager::Dstar::extractPath] Path: " << std::endl;

  for (Eigen::Vector2d p : path) {
    std::cout << "(" << (p.x() * cfg::SystemConfig::Accuracy - cfg::Dimensions::fieldWidth / 2)
              << "," << (p.y() * cfg::SystemConfig::Accuracy - cfg::Dimensions::fieldHeight / 2)
              << ")" << std::endl;

    cfg::SystemConfig::teamOnePath[RobotInd - 1].push_back(Eigen::Vector3d(
        (p.x() * cfg::SystemConfig::Accuracy - cfg::Dimensions::fieldWidth / 2),
        (p.y() * cfg::SystemConfig::Accuracy - cfg::Dimensions::fieldHeight / 2), 0));
  }
}

void Dstar::update(Eigen::Vector2d Start, Eigen::Vector2d Goal) {
  std::cout << "[PathManager::Dstar::update] Update started" << std::endl;

  prevStart = this->Start;
  prevGoal = this->Goal;

  this->Start.Pos = Start.head<2>();
  this->Goal.Pos = Goal.head<2>();
  this->Start = transform(this->Start);
  this->Goal = transform(this->Goal);

  std::cout << "[PathManager::Dstar::update] New Start: (" << this->Start.Pos.x() << ","
            << this->Start.Pos.y() << ")" << std::endl;
  std::cout << "[PathManager::Dstar::update] New Goal: (" << this->Goal.Pos.x() << ","
            << this->Goal.Pos.y() << ")" << std::endl;

  if (prevStart.Pos.x() != this->Start.Pos.x() || prevStart.Pos.y() != this->Start.Pos.y()) {
    km += heuristic(prevStart, this->Start);
    std::cout << "[PathManager::Dstar::update] km updated to " << km << std::endl;
  }

  if (prevGoal.Pos.x() != this->Goal.Pos.x() || prevGoal.Pos.y() != this->Goal.Pos.y()) {
    std::cout << "[PathManager::Dstar::update] Goal changed, reinitializing" << std::endl;

    int pgx = prevGoal.Pos.x();
    int pgy = prevGoal.Pos.y();
    if (constrain(pgx, pgy)) {
      cfg::SystemConfig::grid[pgy][pgx].rhs = INFINITY;
    }

    int gx = this->Goal.Pos.x();
    int gy = this->Goal.Pos.y();
    cfg::SystemConfig::grid[gy][gx].rhs = 0.0f;

    while (!openQueue.empty()) openQueue.pop();

    updateVertex(cfg::SystemConfig::grid[gy][gx]);
    km = 0.0f;
  }

  updateObstacles();
  computeShortestPath();
  extractPath();

  std::cout << "[PathManager::Dstar::update] Update finished" << std::endl;
}

}  // namespace PathManager