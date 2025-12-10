#include "systemConfig.h"

const float cfg::SystemConfig::frameRate = 60;
const float cfg::SystemConfig::wheelDiameter = 0.058;
const float cfg::SystemConfig::distanceFromRobotCenter = 0.1925;
std::vector<cfg::SystemConfig::RobotState> cfg::SystemConfig::PlayerStates;

const std::vector<float> cfg::SystemConfig::wheelAngles = {
    0.523598776f, 2.617993878f, 3.926990817f, 5.497787144f};  // IN RADIANTS

// Friction
const float cfg::SystemConfig::staticBallFrictionCoefficient = 0.5;
const float cfg::SystemConfig::kineticBallFrictionCoefficient = 0.3;
const float cfg::SystemConfig::rollingBallFrictionCoefficient = 0.03;

const float cfg::SystemConfig::staticPlayerFrictionCoefficient = 1.0;
const float cfg::SystemConfig::kineticPlayerFrictionCoefficient = 0.75;
const float cfg::SystemConfig::rollingPlayerFrictionCoefficient = 0.06;

// Mani's Matrix
std::vector<Eigen::Matrix<double, 6, 6>> cfg::SystemConfig::P(cfg::SystemConfig::numRobots,
                                                              Eigen::Matrix<double, 6, 6>::Zero());
static const bool _P_init = ([]() {
  Eigen::Matrix<double, 6, 6> base = Eigen::Matrix<double, 6, 6>::Zero();
  base.diagonal() << 0.05, 0.05, 0.005, 0.02, 0.02, 0.001;
  for (auto& m : cfg::SystemConfig::P) m = base;
  return true;
})();
std::vector<std::vector<cfg::Node>> cfg::SystemConfig::grid;

std::vector<Eigen::Vector3d> cfg::SystemConfig::CameraData;
std::vector<float> cfg::SystemConfig::GyroData;
std::vector<Eigen::Vector4d> cfg::SystemConfig::EncoderData;
std::vector<std::vector<int>> grid;
// Ball config
const float cfg::SystemConfig::ballMass = 0.046f;
const float cfg::SystemConfig::ballRadius = 0.0215f;
const float cfg::SystemConfig::ballAcceleration = 0.1f;
Eigen::Vector2d cfg::SystemConfig::currBallPosition = Eigen::Vector2d(1.0f, 0.0f);
Eigen::Vector2d cfg::SystemConfig::currBallVel = Eigen::Vector2d(0.0f, 0.0f);
const Eigen::Vector2d cfg::SystemConfig::initBallPosition = Eigen::Vector2d(1.0f, 0.0f);
float cfg::SystemConfig::Accuracy =
    0.6;  // 0.02 Meters or 2 CM // The lower this number, the higher the accuracy but slower the
          // execution time // This also corresponds to the jump limit
// Ball possession defaults
bool cfg::SystemConfig::ballHeld = false;
int cfg::SystemConfig::ballHolder = -1;

// Robot config (speed is in m/s)
const float cfg::SystemConfig::robotMass = 1.5f;
const float cfg::SystemConfig::robotRadius = 0.204f;
const float cfg::SystemConfig::playerMaxSpeed = 2.0f;
const float cfg::SystemConfig::playerMaxRotation = 2.0f;
const float cfg::SystemConfig::ballPlayerElasticity = 0.5f;
const float cfg::SystemConfig::playerMaxAcceleration = 1.5f;
const float cfg::SystemConfig::playerMaxOmegaAcceleration = 1.0f;
// const float cfg::SystemConfig::playerMaxShootForce = 0.0f;

// Trajectory config
std::vector<Eigen::Vector3d> cfg::SystemConfig::teamOnePlayerPos = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::numRobots);
  v[0] = Eigen::Vector3d(-3.8f, 0.0f, 0.0f);
  v[1] = Eigen::Vector3d(-0.8f, -0.8f, 0.0f);
  v[2] = Eigen::Vector3d(-0.8f, 0.8f, 0.0f);
  v[3] = Eigen::Vector3d(-0.4f, 0.0f, 0.0f);
  return v;
}();

std::vector<Eigen::Vector3d> cfg::SystemConfig::teamTwoPlayerPos = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::numRobots);
  v[0] = Eigen::Vector3d(3.8f, 0.0f, 0.0f);
  v[1] = Eigen::Vector3d(0.8f, -0.8f, 0.0f);
  v[2] = Eigen::Vector3d(0.8f, 0.3f, 0.0f);
  v[3] = Eigen::Vector3d(0.4f, 0.0f, 0.0f);
  return v;
}();

std::vector<Eigen::Vector3d> cfg::SystemConfig::teamOnePlayerVel = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::numRobots);
  v[0] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  v[1] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  v[2] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  v[3] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  return v;
}();

std::vector<Eigen::Vector3d> cfg::SystemConfig::teamTwoPlayerVel = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::numRobots);
  v[0] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  v[1] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  v[2] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  v[3] = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
  return v;
}();

std::vector<std::vector<Eigen::Vector3d>> cfg::SystemConfig::teamOneWayPoints = {{}, {}, {}, {}};

std::vector<std::vector<Eigen::Vector3d>> cfg::SystemConfig::teamTwoWayPoints = {{}, {}, {}, {}};

std::vector<std::vector<Eigen::Vector3d>> cfg::SystemConfig::teamOnePath = {{}, {}, {}, {}};

std::vector<std::vector<Eigen::Vector3d>> cfg::SystemConfig::teamTwoPath = {{}, {}, {}, {}};
// Wheel RPMs
std::vector<Eigen::Vector4d> cfg::SystemConfig::teamOneWheelRpm = []() {
  std::vector<Eigen::Vector4d> v(SystemConfig::numRobots);
  v[0] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  v[1] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  v[2] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  v[3] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  return v;
}();

std::vector<Eigen::Vector4d> cfg::SystemConfig::teamTwoWheelRpm = []() {
  std::vector<Eigen::Vector4d> v(SystemConfig::numRobots);
  v[0] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  v[1] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  v[2] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  v[3] = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
  return v;
}();

// Team 1 initial formation
std::vector<Eigen::Vector3d> cfg::SystemConfig::teamOneStartFormation = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::numRobots);
  if (SystemConfig::numRobots >= 1) v[0] = Eigen::Vector3d(-3.8f, 0.0f, 0.0f);
  if (SystemConfig::numRobots >= 2) v[1] = Eigen::Vector3d(-0.8f, -0.8f, 0.0f);
  if (SystemConfig::numRobots >= 3) v[2] = Eigen::Vector3d(-0.8f, 0.8f, 0.0f);
  if (SystemConfig::numRobots >= 4) v[3] = Eigen::Vector3d(-0.4f, 0.0f, 0.0f);
  return v;
}();

// Team 2 initial formation
std::vector<Eigen::Vector3d> cfg::SystemConfig::teamTwoStartFormation = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::numRobots);
  if (SystemConfig::numRobots >= 1) v[0] = Eigen::Vector3d(3.8f, 0.0f, 0.0f);
  if (SystemConfig::numRobots >= 2) v[1] = Eigen::Vector3d(0.8f, -0.8f, 0.0f);
  if (SystemConfig::numRobots >= 3) v[2] = Eigen::Vector3d(0.8f, 0.3f, 0.0f);
  if (SystemConfig::numRobots >= 4) v[3] = Eigen::Vector3d(0.4f, 0.0f, 0.0f);
  return v;
}();
