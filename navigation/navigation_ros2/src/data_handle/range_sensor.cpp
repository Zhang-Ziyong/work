#include "data_handle/range_sensor.hpp"

bool RangeSensors::newSensor(const std::string &name,
                             const std::vector<double> &T_os) {
  auto iter = sensors_.find(name);
  if (iter != sensors_.end()) {
    LOG(ERROR) << "sensor: " << name << " is exist.";
    return false;
  }
  LOG(INFO) << "creat " << name;
  std::shared_ptr<RangeSensor> sensor_ptr = std::make_shared<RangeSensor>(T_os);
  sensors_.insert(make_pair(name, sensor_ptr));
  return true;
}

bool RangeSensors::deleteSensor(std::string topic_name) {
  auto iter = sensors_.find(topic_name);
  if (iter == sensors_.end()) {
    return false;
  }
  LOG(INFO) << "stop " << topic_name;
  sensors_.erase(topic_name);
  return true;
}

bool RangeSensors::updateRange(const std::string &name, const double range,
                               uint8_t type) {
  // 超声波数据回调
  auto iter = sensors_.find(name);
  if (iter == sensors_.end()) {
    LOG(ERROR) << "sensor: " << name << " does not exist.";
    return false;
  }

  iter->second->setRange(range, type);
  return true;
}

Eigen::Vector2d RangeSensors::getSensorPoint(const std::string &name) {
  auto iter = sensors_.find(name);
  if (iter == sensors_.end()) {
    LOG(ERROR) << "sensor: " << name << " does not exist.";
    return Eigen::Vector2d(10, 10);
  }
  return iter->second->getPoint();
}

std::vector<Eigen::Vector2d> RangeSensors::getSonarSensorsPoint() {
  std::vector<Eigen::Vector2d> points;
  Eigen::Vector2d point;
  for (auto iter = sensors_.begin(); iter != sensors_.end(); iter++) {
    if (iter->second->getType() == 0) {
      points.push_back(iter->second->getPoint());
    }
  }
  return points;
}

std::vector<Eigen::Vector2d> RangeSensors::getInfraredSensorsPoint() {
  std::vector<Eigen::Vector2d> points;
  Eigen::Vector2d point;
  for (auto iter = sensors_.begin(); iter != sensors_.end(); iter++) {
    if (iter->second->getType() == 1) {
      points.push_back(iter->second->getPoint());
    }
  }
  return points;
}