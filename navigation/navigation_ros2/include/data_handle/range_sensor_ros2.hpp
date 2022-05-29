#ifndef DATA_TYPE_RANGE_SONSER_ROS2_HPP_
#define DATA_TYPE_RANGE_SONSER_ROS2_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include "data_handle/range_sensor.hpp"

class RangeSensorsRos2 {
 public:
  RangeSensorsRos2(const rclcpp::Node::SharedPtr ptr_node);
  ~RangeSensorsRos2();

  void init();

  void rangeSensorCallback(const sensor_msgs::msg::Range::SharedPtr range_msg,
                           const std::string &topic_name);
  void setSonarUpdateFunc(
      const std::function<void(const std::vector<Eigen::Vector2d> &)>
          &update_func);

  void setIRUpdateFunc(
      const std::function<void(const std::vector<Eigen::Vector2d> &)>
          &update_func);

  void startRangeIntput(std::string source);  // 启动某个topic

  void stopRangeIntput(std::string source);  // 关闭某各topic

  void rangeIntputSet(int dir, bool enable);

 private:
  rclcpp::Node::SharedPtr ptr_node_;
  std::map<std::string,
           rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr>
      m_range_sensor_subs_;  ///< 用于订阅range数据
  std::shared_ptr<RangeSensors> ptr_range_sensors_;
  std::function<void(const std::vector<Eigen::Vector2d> &)> update_sonar_func_;
  std::function<void(const std::vector<Eigen::Vector2d> &)> update_ir_func_;
};

#endif