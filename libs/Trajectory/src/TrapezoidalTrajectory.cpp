#include "TrapezoidalTrajectory.h"
namespace Traj {

Trapezoidal_Traj::Trapezoidal_Traj() {
  for (int i = 0; i < int(cfg::SystemConfig::numRobots / 2); i++) init.push_back(false);
}

void Trapezoidal_Traj::SetVelocityFromTraj(int index) {
  index--;
  Eigen::Vector3d Dist;
  bool choice = 0;
  if ((cfg::SystemConfig::teamOneWayPoints[index].size()) == 0) {
    if (!init[index])
      Path.Init(cfg::SystemConfig::teamOnePlayerPos[index].head<2>(),
                cfg::SystemConfig::currBallPosition);
    Path.RobotInd = index + 1;
    init[index] = true;
    if ((cfg::SystemConfig::teamOnePlayerPos[index].head<2>() -
         cfg::SystemConfig::currBallPosition)
            .norm() > 0.3) {
      std::cout << "[Traj::Trapezoidal_Traj::SetVelocityFromTraj] Dist: "
                << (cfg::SystemConfig::teamOnePlayerPos[index].head<2>() -
                    cfg::SystemConfig::currBallPosition)
                       .norm()
                << std::endl;
      Path.CreatePath(cfg::SystemConfig::teamOnePlayerPos[index].head<2>(),
                      cfg::SystemConfig::currBallPosition);
    } else {
      cfg::SystemConfig::teamOnePath[index].clear();
    }

    if (cfg::SystemConfig::teamOnePath[index].size() == 0) {
      std::cout << "[Traj::Trapezoidal_Traj::SetVelocityFromTraj] No Waypoints or Path to follow, "
                   "returning"
                << std::endl;
      return;
    }

    choice = 1;
    Dist = cfg::SystemConfig::teamOnePath[index][0] - cfg::SystemConfig::teamOnePlayerPos[index];
  } else
    Dist =
        cfg::SystemConfig::teamOneWayPoints[index][0] - cfg::SystemConfig::teamOnePlayerPos[index];
  if (Dist.norm() < 0.05) {
    if (choice == 0) {
      cfg::SystemConfig::teamOneWayPoints[index].erase(
          cfg::SystemConfig::teamOneWayPoints[index].begin());
    }
    if (choice == 1) {
      cfg::SystemConfig::teamOnePath[index].erase(cfg::SystemConfig::teamOnePath[index].begin());
    }
    return;
  }
  std::cout << "[Traj::Trapezoidal_Traj::SetVelocityFromTraj] Dist to Goal: "
            << cfg::SystemConfig::teamOnePath[index][0].x() << " "
            << cfg::SystemConfig::teamOnePath[index][0].y() << " is " << Dist.norm() << std::endl;

  double EuclidianDist = Dist.norm();
  double stoppingDist = cfg::SystemConfig::teamOnePlayerVel[index].squaredNorm() /
                        (2 * cfg::SystemConfig::playerMaxAcceleration);
  if (EuclidianDist > stoppingDist) {
    Eigen::Vector3d NewSpeed = cfg::SystemConfig::teamOnePlayerVel[index] +
                               cfg::SystemConfig::playerMaxAcceleration * Dist.normalized() * dt;
    cfg::SystemConfig::teamOnePlayerVel[index] =
        Eigen::Vector3d(std::clamp(float(NewSpeed.x()), -cfg::SystemConfig::playerMaxSpeed,
                                   cfg::SystemConfig::playerMaxSpeed),
                        std::clamp(float(NewSpeed.y()), -cfg::SystemConfig::playerMaxSpeed,
                                   cfg::SystemConfig::playerMaxSpeed),
                        std::clamp(float(NewSpeed.z()), -cfg::SystemConfig::playerMaxRotation,
                                   cfg::SystemConfig::playerMaxRotation));
  } else {
    auto Decelerate = [](float velocity, float maxAcc) {
      float decel = std::min(std::abs(velocity), maxAcc);
      if (velocity > 0) decel *= -1;
      return velocity + decel * dt;
    };

    cfg::SystemConfig::teamOnePlayerVel[index].x() = Decelerate(
        cfg::SystemConfig::teamOnePlayerVel[index].x(), cfg::SystemConfig::playerMaxAcceleration);
    cfg::SystemConfig::teamOnePlayerVel[index].y() = Decelerate(
        cfg::SystemConfig::teamOnePlayerVel[index].y(), cfg::SystemConfig::playerMaxAcceleration);
    cfg::SystemConfig::teamOnePlayerVel[index].z() =
        Decelerate(cfg::SystemConfig::teamOnePlayerVel[index].z(),
                   cfg::SystemConfig::playerMaxOmegaAcceleration);
  };
}
}  // namespace Traj
