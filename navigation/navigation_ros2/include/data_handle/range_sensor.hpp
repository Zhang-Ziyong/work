#ifndef DATA_TYPE_RANGE_SONSER_HPP_
#define DATA_TYPE_RANGE_SONSER_HPP_
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "eigen3/Eigen/Core"
#include "log.hpp"

class RangeSensor {
 public:
  RangeSensor(const std::vector<double> &T_os) {
    if (T_os.size() != 3) {
      LOG(ERROR) << "set range sensor tf wrong, tf size is " << T_os.size();
    } else {
      LOG(INFO) << "set range sensor tf: (" << T_os[0] << ", " << T_os[1]
                << ", " << T_os[2] << ")";
      T_os_ = T_os;
      point[0] = 10;
      point[1] = 10;
    }
  }

  ~RangeSensor() = default;

  inline void setRange(const double range, uint8_t radiation_type) {
    point[0] = T_os_[0] + range * std::cos(T_os_[2]);
    point[1] = T_os_[1] + range * std::sin(T_os_[2]);
    type = radiation_type;
  }
  inline Eigen::Vector2d getPoint() { return point; }
  inline uint8_t getType() { return type; }

 private:
  std::vector<double> T_os_;
  Eigen::Vector2d point;
  uint8_t type;
};

class RangeSensors {
 public:
  RangeSensors() = default;
  ~RangeSensors() = default;

  bool newSensor(const std::string &name, const std::vector<double> &T_os);
  bool deleteSensor(std::string topic_name);
  bool updateRange(const std::string &name, const double range, uint8_t type);

  Eigen::Vector2d getSensorPoint(const std::string &name);

  std::vector<Eigen::Vector2d> getSonarSensorsPoint();
  std::vector<Eigen::Vector2d> getInfraredSensorsPoint();

 private:
  std::map<std::string, std::shared_ptr<RangeSensor>> sensors_;
};

#endif