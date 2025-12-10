#include "Game.h"
#include "dropDownUI.h"

namespace Controller {

Game::Game(QWidget* parent) : QWidget(parent) {
  std::cout << "[Controller::Game::Game] Startup Successfull" << std::endl;
  for (int _ = 0; _ < int(cfg::SystemConfig::numRobots / 2); ++_) {
    cfg::SystemConfig::PlayerStates.push_back(cfg::SystemConfig::RobotState::Manual);
  }
  this->resize(QGuiApplication::primaryScreen()->geometry().size());
  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this]() { this->update(); });
  timer->start(16);
}

void Game::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  cfg::Dimensions::screenWidth = width();
  cfg::Dimensions::screenHeight = height();
  Transformation::instance().transformation(&painter, rect());
  // comms::receiveGrSimRobots();
  keyControls();
  drawField(&painter);
  ManagePlayers(&painter);
  drawBall(&painter);
  drawStats(&painter);

  Hardware.update();
  painter.save();
  painter.resetTransform();
  DropDownUI::Draw(painter);
  painter.restore();
}

void Game::mousePressEvent(QMouseEvent* event) {
  if (DropDownUI::HandleClick(this, event->x(), event->y())) {
    update();
  }
}

void Game::drawField(QPainter* painter) { this->Field.CreateField(painter); }
void Game::ManagePlayers(QPainter* painter) { this->Players_.ManagePlayers(painter, PlayerKeys); }
void Game::drawBall(QPainter* painter) { this->Ball_.drawBall(painter); }
void Game::drawStats(QPainter* painter) {
  this->Stats.drawStats(painter);
  Stats.PLayerID = SelectedPlayerId;
}

void Game::keyPressEvent(QKeyEvent* event) { handleInsertKey(event->key()); }
void Game::keyReleaseEvent(QKeyEvent* event) { handleRemoveKey(event->key()); }
void Game::keyControls() { handlePlayer(); }
void Game::handleInsertKey(int key) { PlayerKeys.insert(key); }
void Game::handleRemoveKey(int key) { PlayerKeys.remove(key); }

}  // namespace Controller
