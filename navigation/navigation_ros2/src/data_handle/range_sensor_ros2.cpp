#include "data_handle/range_sensor_ros2.hpp"

RangeSensorsRos2::RangeSensorsRos2(const rclcpp::Node::SharedPtr ptr_node) {
  ptr_node_ = ptr_node;
  ptr_range_sensors_ = std::make_shared<RangeSensors>();
}

RangeSensorsRos2::~RangeSensorsRos2() {}

void RangeSensorsRos2::init() {
  std::string topics;
  ptr_node_->get_parameter_or("range_sensor.topics", topics, std::string(""));

  if (!topics.empty()) {
    std::stringstream ss(topics);

    std::string topic_name;
    while (ss >> topic_name) {
      LOG(INFO) << "subscribe range_sensor: " << topic_name;

      std::vector<double> sensor_tf;
      ptr_node_->get_parameter_or("range_sensor." + topic_name + "_sensor_tf",
                                  sensor_tf, sensor_tf);

      ptr_range_sensors_->newSensor(topic_name, sensor_tf);
      m_range_sensor_subs_.insert(std::make_pair(
          topic_name,
          ptr_node_->create_subscription<sensor_msgs::msg::Range>(
              topic_name, rclcpp::QoS(10).best_effort(),
              [this,
               topic_name](const sensor_msgs::msg::Range::SharedPtr range_msg) {
                rangeSensorCallback(range_msg, topic_name);
              })));
    }
  } else {
    return;
  }
}

void RangeSensorsRos2::setSonarUpdateFunc(
    const std::function<void(const std::vector<Eigen::Vector2d> &)>
        &update_func) {
  update_sonar_func_ = update_func;
}

void RangeSensorsRos2::setIRUpdateFunc(
    const std::function<void(const std::vector<Eigen::Vector2d> &)>
        &update_func) {
  update_ir_func_ = update_func;
}

void RangeSensorsRos2::rangeSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr range_msg,
    const std::string &topic_name) {
  // LOG(INFO) << "sonar callback";
  double range = range_msg->range;
  if (range < range_msg->min_range) {
    range = range_msg->max_range;
  }
  if (range > range_msg->max_range) {
    range = range_msg->max_range;
  }
  ptr_range_sensors_->updateRange(topic_name, range, range_msg->radiation_type);
  if (update_sonar_func_ != nullptr) {
    update_sonar_func_(ptr_range_sensors_->getSonarSensorsPoint());
  }
  if (update_ir_func_ != nullptr) {
    update_ir_func_(ptr_range_sensors_->getInfraredSensorsPoint());
  }
}

void RangeSensorsRos2::startRangeIntput(std::string topic_name) {
  if (!m_range_sensor_subs_.count(topic_name)) {
    LOG(INFO) << "subscribe range_sensor: " << topic_name;
    std::vector<double> sensor_tf;
    ptr_node_->get_parameter_or("range_sensor." + topic_name + "_sensor_tf",
                                sensor_tf, sensor_tf);

    ptr_range_sensors_->newSensor(topic_name, sensor_tf);
    m_range_sensor_subs_.insert(std::make_pair(
        topic_name,
        ptr_node_->create_subscription<sensor_msgs::msg::Range>(
            topic_name, rclcpp::QoS(10).best_effort(),
            [this,
             topic_name](const sensor_msgs::msg::Range::SharedPtr range_msg) {
              rangeSensorCallback(range_msg, topic_name);
            })));
  } else {
    LOG(INFO) << topic_name << " has already begun";
  }
}

void RangeSensorsRos2::stopRangeIntput(std::string topic_name) {
  LOG(INFO) << "stop " << topic_name;
  ptr_range_sensors_->deleteSensor(topic_name);
  m_range_sensor_subs_.erase(topic_name);
}

void RangeSensorsRos2::rangeIntputSet(int dir, bool enable) {
  std::string topics;
  ptr_node_->get_parameter_or("range_sensor.topics", topics, std::string(""));
  if (topics.empty()) {
    // 未订阅话题  不操作
    return;
  }

  if (enable) {
    std::string topic_name;
    std::stringstream ss(topics);
    while (ss >> topic_name) { startRangeIntput(topic_name); }
  } else {
    switch (dir) {
      case 0: {
        // 前
        stopRangeIntput("sonar_fl");
        stopRangeIntput("sonar_fr");
        stopRangeIntput("sonar_lf");

        stopRangeIntput("right_tof_sensor");
        stopRangeIntput("left_tof_sensor");
      } break;
      case 1: {
        // 右
        stopRangeIntput("sonar_fr");
        stopRangeIntput("sonar_rb");
        stopRangeIntput("sonar_rf");
        stopRangeIntput("sonar_br");
        stopRangeIntput("right_tof_sensor");
      } break;
      case 2: {
        // 后
        stopRangeIntput("sonar_rb");
        stopRangeIntput("sonar_bl");
        stopRangeIntput("sonar_br");
        stopRangeIntput("sonar_lb");
      } break;
      case 3: {
        // 左
        stopRangeIntput("sonar_fl");
        stopRangeIntput("sonar_lb");
        stopRangeIntput("sonar_lf");
        stopRangeIntput("sonar_bl");
        stopRangeIntput("left_tof_sensor");
      } break;
      default: {
        // 全部
        std::string topic_name;
        std::stringstream ss(topics);
        while (ss >> topic_name) { stopRangeIntput(topic_name); }
      } break;
    }
  }
}