#include <iostream>
#include <fstream>
#include <vector>
#include <jsoncpp/json/json.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "marker_map.hpp"
#include "costmap_2d.hpp"
#include "perception_utility.hpp"

using namespace std;
using namespace CVTE_BABOT;

rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr puber = nullptr;
nav_msgs::msg::OccupancyGrid grid_map;

void pubProcess() {
  rclcpp::Rate loop_rate(1.0);
  while (rclcpp::ok()) {
    cout << "pub ..." << endl;
    puber->publish(grid_map);
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc != 2) {
    cout << "please run: ./node marker.yaml" << endl;
  }

  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("strip_marker_node");

  // 读取本地文件
  ifstream ifs;
  ifs.open(argv[1]);
  assert(ifs.is_open());
  Json::Reader reader;
  Json::Value root;
  if (!reader.parse(ifs, root, false)) {
    cout << "reader parse error: " << strerror(errno) << endl;
    return 0;
  }

  // 读取地图参数
  double map_res, map_ori_x, map_ori_y;  // 地图分辨率，原点
  int map_height, map_width;             // 地图尺寸
  Json::Value map_param = root["map"];
  map_res = map_param["resolution"].asDouble();
  map_ori_x = map_param["origin_x"].asDouble();
  map_ori_y = map_param["origin_y"].asDouble();
  map_height = map_param["height"].asDouble();
  map_width = map_param["width"].asDouble();
  cout << "map param: " << endl;
  cout << "resolution: " << map_res << "   origin: [" << map_ori_x << ","
       << map_ori_y << "]   size: [" << map_width << "," << map_height << "]"
       << endl;

  // 读取减速框顶点
  vector<vector<vector<double>>> polygons;
  Json::Value strips_param = root["strips"];
  size_t strip_size = strips_param.size();
  for (int i = 0; i < strip_size; i++) {
    Json::Value strip = strips_param[i];
    Json::Value strip_points = strip["points"];
    size_t points_size = strip_points.size();
    vector<vector<double>> polygon;
    polygon.clear();
    cout << "polygon " << i << ": ";
    for (int j = 0; j < points_size; j++) {
      Json::Value point = strip_points[j];
      vector<double> tp(2, 0.0);
      tp[0] = point["x"].asDouble();
      tp[1] = point["y"].asDouble();
      polygon.push_back(tp);
      cout << "[" << tp[0] << "," << tp[1] << "]   ";
    }
    cout << endl;

    if (polygon.size() < 3) {
      cout << "polygon size " << polygon.size() << " less than 3 !" << endl;
      continue;
    }
    polygons.push_back(polygon);
  }

  // 在marker地图中绘制并填充减速带
  MarkerMap marker;
  marker.resetMap(map_width, map_height, map_res, map_ori_x, map_ori_y);
  marker.setStripsMarker(strips_param);

  // 显示绘制结果
  cout << "start paint ................" << endl;
  std::shared_ptr<Costmap2d> ptr_marker_map = marker.getMarkerMap();
  generateCostmapMsg(ptr_marker_map, grid_map);
  cout << "finish :)" << endl;

  puber = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "marker_map", rclcpp::QoS(10).best_effort());

  thread pub_thread(pubProcess);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);

  pub_thread.join();

  return 0;
}