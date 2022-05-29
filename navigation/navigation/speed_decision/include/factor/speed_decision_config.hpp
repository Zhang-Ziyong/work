/*
 * @Author: your name
 * @Date: 2021-06-02 14:28:05
 * @LastEditTime: 2021-06-28 16:13:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /navigation/navigation/speed_decision/include/speed_decision_config.hpp
 */
#ifndef SPEED_DECISION_CONFIG_HPP_
#define SPEED_DECISION_CONFIG_HPP_

#include <vector>
#include <map>
#include <memory>
#include "eigen3/Eigen/Core"

enum DeriveType { FRONT, REAR };
enum BoundArea { TOPLEFT = 0, UNDERRIGHT };
enum SpeedRadio { STOP = 0, LEVEL1, LEVEL2, LEVEL3, LEVEL4 };
enum SpeedDir { SHAPE_FRONT, SHAPE_BACK, SHAPE_LEFT, SHAPE_RIGHT };
enum RecoveDir { RECOVER_FRONT, RECOVER_BACK, RECOVER_LEFT, RECOVER_RIGHT };

typedef struct StopShapeConfig {
  std::string key;             // 名称
  std::vector<double> params;  // 圆-->size= 3(center radius)
                               // 矩形--> size= 4(left_top right_top)
  double start_theta = 0;      // 检测角度
  double end_theta = 0;
} StopShapeConfig;

struct SpeedDecisionBaseConfig {
  SpeedDecisionBaseConfig() {
    // 默认参数
    T_bl << 1, 0, 0.85, 0, 1, 0;
    path_slow_length = 5.0;

    max_v = 1.0;
    max_w = 0.5;
    min_v = 0.02;
    min_w = 0.02;

    max_v_inc = 0.025;
    max_w_inc = 0.025;
    max_v_dec = -0.1;
    max_w_dec = -0.1;

    extra_stop_v_radio = 0.5;
    extra_stop_w_radio = 0.5;

    slow_cost = 40.0;
    stop_cost = 100.0;

    max_slope = 15.0 * M_PI / 180.0;
    min_slope = 3.0 * M_PI / 180.0;

    max_camera_distance = 10.0;

    costmap_traverse_inc = 0.1;

    driver_type = 1;
  }

  // TODO: 在sensorHub中删除
  Eigen::Matrix<double, 2, 3> T_bl;  // 激光到baselink的长度
  double path_slow_length;           // 路径障碍物减速距离

  double max_v;  // 机器运行速度阈值
  double max_w;
  double min_v;
  double min_w;

  double max_v_inc;  // 加速度阈值
  double max_w_inc;
  double max_v_dec;
  double max_w_dec;

  double extra_stop_v_radio;  // 停障框扩大系数
  double extra_stop_w_radio;  // 公式： inc_d = v * extra_radio;

  double slow_cost;  // 减速代价值 cost_map
  double stop_cost;  // 停障代价值

  double max_slope;  // 倾斜减速 角度限幅
  double min_slope;

  double max_camera_distance;

  double costmap_traverse_inc;  // cost_map 遍历增量 --> stopShape resolution

  int driver_type;  // 前驱 -1 后驱 1

  std::map<std::string, std::vector<StopShapeConfig>> m_shape;
};

struct SpeedDecisionResult {
  SpeedDecisionResult()
      : max_v(1.0),
        max_w(0.5),
        real_speed_controller("mission_manager"),
        speed_radio(LEVEL4) {}
  double max_v;
  double max_w;
  double double_speed_radio;
  std::string real_speed_controller;
  SpeedRadio speed_radio;
};

#endif
