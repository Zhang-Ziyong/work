#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <glog/logging.h>

#include "manager_ros2/map_manager_ros2.hpp"
#include "narrow_recognition.hpp"

#include "perception_utility.hpp"

using namespace std;
using namespace CVTE_BABOT;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("test_narrow_recognition_node");

  // google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);  // 配置日志打印级别

  // 初始化发布器
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_costmap_pub =
      node->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "raw_costmap", rclcpp::QoS(10).best_effort());

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mark_map_pub =
      node->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "mark_map", rclcpp::QoS(10).best_effort());

  // 读取代价地图参数
  string map_path;         // 地图路径
  double inscribe_radius;  // 内切半径
  double inflate_scale;    // 膨胀梯度因子

  std::shared_ptr<CostmapArchAdapter> ptr_costmap_arch =
      CostmapArchAdapter::getPtrInstance();
  ptr_costmap_arch->updateParameter();
  ptr_costmap_arch->systemInit();
  ptr_costmap_arch->startCostmapTimer();
  auto costmap_node = ptr_costmap_arch->getNodeHander();

  costmap_node->get_parameter_or("map_path", map_path, std::string(""));
  costmap_node->get_parameter_or("inflation_layer.cost_scaling_factor",
                                 inflate_scale, 1.0);
  inscribe_radius = CostmapMediator::getPtrInstance()->getInscribedRadius();
  LOG(INFO) << "map_path->" << map_path << " inflate_scale->" << inflate_scale
            << " inscribe_radius->" << inscribe_radius;

  // 读取原始代价地图
  std::shared_ptr<Costmap2d> static_costmap = nullptr;
  if (!CostmapMediator::getPtrInstance()->getData("static_costmap",
                                                  static_costmap)) {
    LOG(ERROR) << "cant get static costmap !!!";
    return 0;
  }

  // 识别窄道
  std::shared_ptr<Costmap2d> mark_map = nullptr;
  const double narrow_width = 1.3;
  NarrowRecognition nr(narrow_width, inflate_scale, inscribe_radius);
  nr.recognition(static_costmap, mark_map);

  // LOG(INFO) << "static map size: " << int(static_costmap->getSizeInCellsX())
  //           << " " << int(static_costmap->getSizeInCellsY())
  //           << "   resolution: " << static_costmap->getResolution();

  // 循环发布
  rclcpp::Rate loop_rate(10.0);
  while (rclcpp::ok()) {
    // 将原始代价地图转换成消息并发布
    nav_msgs::msg::OccupancyGrid static_costmap_msg;
    generateCostmapMsg(static_costmap, static_costmap_msg);
    raw_costmap_pub->publish(static_costmap_msg);

    // 将标记地图转换成消息并发布
    nav_msgs::msg::OccupancyGrid mark_map_msg;
    generateCostmapMsg(mark_map, mark_map_msg);
    mark_map_pub->publish(mark_map_msg);

    rclcpp::spin_some(node);
    // rclcpp::spin_some(costmap_node);
    loop_rate.sleep();
  }

  return 0;
}
