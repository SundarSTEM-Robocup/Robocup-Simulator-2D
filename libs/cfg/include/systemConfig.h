#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Eigen/Dense>

namespace cfg {
struct Node {
  Eigen::Vector2d Pos;
  float g = INFINITY;
  float rhs = INFINITY;
  Eigen::Vector2d Key = Eigen::Vector2d(INFINITY, INFINITY);
  bool obstacle = false;
  bool operator>(const Node& other) const {
    if (Key.x() != other.Key.x()) return Key.x() > other.Key.x();
    return Key.y() > other.Key.y();
  }
};
struct SystemConfig {
  static const float wheelRPM;
  static const float frameRate;
  static const float wheelDiameter;
  static constexpr int numRobots = 8;
  static const float distanceFromRobotCenter;
  enum class RobotState { Autonomous, Manual };
  static const std::vector<float> wheelAngles;
  static std::vector<RobotState> PlayerStates;
  static std::vector<Eigen::Matrix<double, 6, 6>> P;
  static std::vector<std::vector<Node>> grid;
  static float Accuracy;

  // Friction
  static const float staticBallFrictionCoefficient;
  static const float kineticBallFrictionCoefficient;
  static const float rollingBallFrictionCoefficient;

  static const float staticPlayerFrictionCoefficient;
  static const float kineticPlayerFrictionCoefficient;
  static const float rollingPlayerFrictionCoefficient;

  // Ball
  static const float ballMass;
  static const float ballRadius;
  static const float ballAcceleration;
  static Eigen::Vector2d currBallPosition;
  static Eigen::Vector2d currBallVel;
  static const Eigen::Vector2d initBallPosition;

  // Robots
  static const float robotMass;
  static const float robotRadius;
  static const float playerMaxSpeed;
  static const float playerMaxRotation;
  static const float ballPlayerElasticity;
  static const float playerMaxAcceleration;
  static const float playerPlayerElasticity;
  static const float playerMaxOmegaAcceleration;
  // static const float playerMaxShootForce;

  // Trajectory
  static std::vector<Eigen::Vector3d> CameraData;
  static std::vector<float> GyroData;
  static std::vector<Eigen::Vector4d> EncoderData;

  static std::vector<Eigen::Vector3d> teamOnePlayerPos;
  static std::vector<Eigen::Vector3d> teamTwoPlayerPos;
  static std::vector<Eigen::Vector3d> teamOnePlayerVel;
  static std::vector<Eigen::Vector3d> teamTwoPlayerVel;
  static std::vector<std::vector<Eigen::Vector3d>> teamOneWayPoints;
  static std::vector<std::vector<Eigen::Vector3d>> teamTwoWayPoints;
  static std::vector<std::vector<Eigen::Vector3d>> teamOnePath;
  static std::vector<std::vector<Eigen::Vector3d>> teamTwoPath;
  // Robot Team
  static std::vector<Eigen::Vector4d> teamOneWheelRpm;
  static std::vector<Eigen::Vector4d> teamTwoWheelRpm;
  static std::vector<Eigen::Vector3d> teamOneStartFormation;
  static std::vector<Eigen::Vector3d> teamTwoStartFormation;

  // Ball possession
  static bool ballHeld;
  static int ballHolder;  // player ID who holds the ball, -1 if none
};
}  // namespace cfg

#endif
