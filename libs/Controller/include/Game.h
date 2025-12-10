#ifndef GAME_H
#define GAME_H
#include <QApplication>
#include <QWidget>
#include <QPainterPath>
#include <QPainter>
#include <QScreen>
#include <QTimer>
#include <QKeyEvent>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "dimensions.h"
#include "transformation.h"
#include "ball.h"
#include "playingField.h"
#include "Robotstats.h"
#include "playerManager.h"
#include "Sensor.h"
// #include "DrawStuff.h"
// #include "grsim_interface.h"
#define To_Be_Implemented int
#define dt 1 / cfg::SystemConfig::frameRate
namespace Controller {
enum class State { Kickoff, Playing, Paused, Foul, Finished };

class Game : public QWidget {
 private:
  State gameState = State::Kickoff;
  Player::PlayerManager Players_;
  Ball::Ball Ball_;
  UI Field;
  RbSt Stats;
  HW::SensorManager Hardware;
  void drawStats(QPainter* painter);
  void drawField(QPainter* painter);
  void ManagePlayers(QPainter* painter);
  void drawBall(QPainter* painter);
  void paintEvent(QPaintEvent*) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void handleInsertKey(int Key);
  void handleRemoveKey(int Key);
  void keyControls();
  void handlePlayer();
  QSet<int> PlayerKeys;
  int SelectedPlayerId = 0;

 public:
  Game(QWidget* parent = nullptr);
};

}  // namespace Controller

#endif