#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "cvte_sensor_msgs/msg/sensor_switch.hpp"

rclcpp::Publisher<cvte_sensor_msgs::msg::SensorSwitch>::SharedPtr switch_pub =
    nullptr;
geometry_msgs::msg::PointStamped::SharedPtr last_click_msg = nullptr;
geometry_msgs::msg::PointStamped::SharedPtr click_msg = nullptr;

void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  click_msg = msg;
}

void process() {
  int number = 10;
  rclcpp::Rate loop_rate(10.0);
  while (rclcpp::ok()) {
    if (!click_msg) {
      loop_rate.sleep();
      continue;
    }

    cvte_sensor_msgs::msg::SensorSwitch ss;
    ss.header.stamp = click_msg->header.stamp;
    ss.status = 0;
    if (click_msg != last_click_msg) {
      if (number <= 0) {
        last_click_msg = click_msg;
        number = 10;
      }
      ss.status = 1;
      number--;
      std::cout << "collision switch is crash !!!" << std::endl;
    }
    switch_pub->publish(ss);

    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("collision_switch_node");

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub =
      node->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", rclcpp::QoS(1).best_effort(), clickPointCallback);

  switch_pub = node->create_publisher<cvte_sensor_msgs::msg::SensorSwitch>(
      "/collision_switch", rclcpp::QoS(10).best_effort());

  std::thread process_thread(process);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);

  process_thread.join();

  return 0;
}