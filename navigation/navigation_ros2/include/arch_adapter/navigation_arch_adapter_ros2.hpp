/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file navigation_arch_adapter_ros2.hpp
 *
 *@brief
 * 架构相关接口的基类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev
 *@data 2020-01-07
 ************************************************************************/
#ifndef __NAVIGATION_ARCH_ADAPTER_ROS2_H
#define __NAVIGATION_ARCH_ADAPTER_ROS2_H

#include <jsoncpp/json/json.h>
#include <pcl/point_cloud.h>
#include <pcl/register_point_struct.h>

#include <boost/mpl/identity.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "clean_decision.hpp"
#include "data_handle/range_sensor_ros2.hpp"
#include "mission_manager_msgs/srv/mission_manager.hpp"
#include "navigation_arch_target.hpp"
#include "planner_utils.hpp"
#include "pnc_map.hpp"
#include "robot_info.hpp"
#include "robot_trajectory_msgs/msg/robot_trajectory.hpp"
#include "speed_decision_factory.hpp"
#include "trajectory.hpp"
#include "arch_adapter/zmq_server.hpp"
#include "public_parameters/PublicParameters.hpp"

namespace CVTE_BABOT {
class LogicController;
class CostmapArchAdapter;
class PathOptimize;
class NavigationParameters;
class NavigationParamsRos2;
class NavigationMediator;

enum class NavigationState {
  UINIT,
  IDLE,
  ELEVATOR_DONE,
  RUNNING,
  PAUSE,
  ERROR,
};

enum ElevatorState {
  INELEVATOR,
  OUTELEVATOR,
  HALFELEVATOR,
};

// enum PathType { CLOSE, FOLLOW, SHOPE };

enum class NavigationErrorCode {
  NO,
  ODOM_TIMEOUT,
  LOCALIZATION_TIMEOUT,
  POINT_OCCUPY,
  PATH_OCCUPY,
  PIT_OCCUPY,
  OBSTACLE_STOP,
  RECOVERY_ERROR,
  LOCALIZATION_STOP,
  PATH_FARAWAY,
  GOAL_FARAWAY,
  OVERTIME
};

struct MapGridCostPoint {
  float x;
  float y;
  float z;
  float path_cost;
  float goal_cost;
  float occ_cost;
  float total_cost;
};

class NavigationArchAdapter final : public NavigationArchTarget {
 public:
  NavigationArchAdapter();
  ~NavigationArchAdapter();

  void spin() final;
  bool systemInit() final;
  bool stopNavigation() final;
  bool startNavigation() final;
  bool updateParameter() final;
  bool startWithDefaultNavigation() final;

 private:
  rclcpp::Node::SharedPtr node_ = nullptr;  ///< 用于收发数据的节点
  rclcpp::Node::SharedPtr node_service_ = nullptr;  ///< 用于接收服务的节点

  rclcpp::Node::SharedPtr node_scan_ = nullptr;
  rclcpp::Node::SharedPtr node_odom_ = nullptr;
  rclcpp::Node::SharedPtr node_point_cloud_ = nullptr;
  rclcpp::Node::SharedPtr node_infrared_ = nullptr;
  rclcpp::Node::SharedPtr node_pose_ = nullptr;

  //   rclcpp::Node::SharedPtr node_planner_;  ///< 用于收发数据的节点
  //   rclcpp::Node::SharedPtr node_controller_;  ///< 用于导航规划的定时器节点
  rclcpp::TimerBase::SharedPtr planner_timer_ =
      nullptr;  ///< 路径规划规划定时器
  rclcpp::TimerBase::SharedPtr controller_timer_ =
      nullptr;  ///< 计算局部控制定时器

  std::shared_ptr<NavigationParameters> ptr_navigation_params_ = nullptr;
  std::shared_ptr<NavigationParamsRos2> navigation_params_ros2_ = nullptr;
  std::shared_ptr<LogicController> ptr_logic_controller_ = nullptr;
  std::shared_ptr<PathOptimize> ptr_optimize_path_ = nullptr;
  std::shared_ptr<SpeedDecisionBase> ptr_speed_decision_ = nullptr;
  std::shared_ptr<RangeSensorsRos2> ptr_range_sensors_handle_ = nullptr;
  std::shared_ptr<CleanDecision> ptr_clean_decision_ = nullptr;
  std::shared_ptr<PncMap> ptr_pnc_map_ = nullptr;

  std::chrono::time_point<std::chrono::system_clock> last_odom_time_;
  std::chrono::time_point<std::chrono::system_clock> last_localization_time_;

  NavigationState state_ = NavigationState::UINIT;
  NavigationErrorCode error_code_ = NavigationErrorCode::NO;
  ElevatorState elevator_state_ = ElevatorState::OUTELEVATOR;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ =
      nullptr;  ///< 用于订阅odom数据
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      localization_sub_ = nullptr;  ///< 用于订阅localization数据
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_ =
      nullptr;  ///< 用于订阅路径数据
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_ =
      nullptr;  ///< 用于订阅目标点数据
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_ =
      nullptr;  ///< 用于订阅雷达数据
  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr psd_sub_ =
      nullptr;  ///< 用于订阅psd数据
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr psd_sub_front_ =
      nullptr;  ///< 用于订阅psd数据
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr psd_sub_back_ =
      nullptr;  ///< 用于订阅psd数据
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      camera_cloud_sub_ = nullptr;  ///< 用于订阅psd数据
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_path_sub_ =
      nullptr;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr infrared_sub_ =
      nullptr;  //< 用于订阅避障红外数据

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_ =
      nullptr;  ///< 用于发布机器人局部规划，用于可视化
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_ =
      nullptr;  ///< 用于发布机器人全局规划，用于可视化
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_plan_pub_ =
      nullptr;  ///< 用于发布机器人全局规划，用于可视化
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_ =
      nullptr;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr global_pose_pub_ =
      nullptr;  ///< 用于发布任务路径中的每一点，包含角度信息，用于可视化
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr local_pose_pub_ =
      nullptr;  ///< 用于发布任务路径中的每一点，包含角度信息，用于可视化
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      ori_local_pose_pub_ =
          nullptr;  ///< 用于发布任务路径中的每一点，包含角度信息，用于可视化
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr remain_path_pub_ =
      nullptr;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr carrot_pub_ =
      nullptr;  ///< 用于发布PID算法规划时的胡萝卜跟踪点，用于可视化
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr area_pub_ = nullptr;
  rclcpp::Service<mission_manager_msgs::srv::MissionManager>::SharedPtr
      mission_manager_server_ = nullptr;  ///< 用于接收任务管理发过来的命令
  std::shared_ptr<ZmqService> zmq_server_ = nullptr;
  PublicParameters params_;
  std::string zmq_server_addr_;
  unsigned short navigation_server_port_;

  std::thread *controller_thread_ = nullptr;
  std::thread *planner_thread_ = nullptr;
  std::thread *hdmap_thread_ = nullptr;
  std::thread *global_replan_thread_ = nullptr;
  std::thread *update_costmap_thread_ = nullptr;

  std::shared_ptr<CostmapArchAdapter> ptr_costmap_arch_ =
      nullptr;                                    ///< costmap接口
  rclcpp::Clock::SharedPtr ptr_clock_ = nullptr;  ///< 时钟指针

  bool is_debug_mode_ = true;          ///< 是否调试模式
  bool use_default_path_ = false;      ///< 是否使用默认路径
  bool use_simulator_env_ = false;     ///< 是否使用仿真环境
  bool is_abs_reach_ = true;           ///< 是否等待到达终点
  bool stop_thread_ = false;           //< 是否结束线程
                                       //   bool in_elevator_ = false;
  int replan_count_ = 0;               ///< 记录重规划次数
  double planner_frequency_ = 1.0;     ///< 重规划频率（读参）
  double controller_frequency_ = 1.0;  ///< 控制频率（读参）
  double x_offset_;

  std::mutex navi_state_mutex_;          ///< 导航状态锁
  std::mutex navi_error_mutex_;          ///< 导航异常锁
  std::mutex odom_time_mutex_;           ///< 里程时间锁
  std::mutex localization_time_mutex_;   ///< 定位时间锁
  std::mutex hd_map_mutex_;              ///< 语义地图锁
  std::string s_pose_topic_ = "";        ///< 默认定位数据名
  std::string s_control_topic_ = "";     ///< 默认发布控制数据名
  std::string default_path_file_ = "";   ///< 默认路径文件
  std::string s_edge_front_topic_ = "";  ///< 贴边传感器名称
  std::string s_edge_back_topic_ = "";   ///< 后方贴边传感器名称

  std::vector<geometry_msgs::msg::PoseStamped>
      local_plan_;  ///< 用于调试的局部规划结果
  std::vector<geometry_msgs::msg::PoseStamped>
      global_plan_;  ///< 用于调试的全局规划结果
  std::vector<Pose2d> global_path_;

  std::string robot_tpye_ = "";  ///< 机器人类型

  AreaType cur_area_type_;
  AreaType last_area_type_;

  std::map<AreaType, std::string> map_area_types_;

  std::string cur_static_map_md5_{
      ""};  ///< 当前导航正在使用的地图md5值(可作为地图的唯一标志)

  std::map<int, MISSIONTYPE> map_int_mission_ = {{0, MISSIONTYPE::EDGE},
                                                 {1, MISSIONTYPE::TRACKING},
                                                 {2, MISSIONTYPE::SLOPE},
                                                 {3, MISSIONTYPE::ELVATOR}};

  void setStripMarker(const Json::Value &rev_data, Json::Value &ack_data);

  bool resetStaticCostmap(const std::string &map_path,
                          const std::string &osm_path, const std::string &md5);
  bool resetHdmap(const std::string &map_path);
  void defaultPath(std::vector<Pose2d> &reference_path);
  bool stringToJson(const std::string &str_data, Json::Value &pose);

  void localizationDataCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
          localization_msg);
  void goalPoseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg);
  void OdomDataCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void goalPathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void psdCallback(const std_msgs::msg::UInt32MultiArray::SharedPtr arrary);
  void psdCallbackFront(const sensor_msgs::msg::Range::SharedPtr psd);
  void psdCallbackBack(const sensor_msgs::msg::Range::SharedPtr psd);

  void cameraCloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);

  void pubPathCallback(const std_msgs::msg::Empty::SharedPtr update);
  void infraredObstacleCallback(
      const sensor_msgs::msg::Range::SharedPtr infrared_range);

  void updateCostMapTimerCallback();
  void plannerTimerCallback();
  void controllerTimerCallback();
  void globalReplanTimerCallback();
  void hdmapTimerCallback();
  void dealAreaType(AreaType area_type);
  void clearReplanCount() { replan_count_ = 0; }
  void missionManagerPauseNavi();
  void missionManagerSetCarResult(const Json::Value &rev_data,
                                  Json::Value &ack_data);
  void missionManagerSetGoalPose(const Json::Value &rev_data,
                                 Json::Value &ack_data);
  void missionManagerSetGoalPath(const Json::Value &rev_data,
                                 Json::Value &ack_data);
  void missionManagerGetCurrentState(const Json::Value &,
                                     Json::Value &ack_data);
  void missionManagerSetFullGoalPath(const Json::Value &,
                                     Json::Value &ack_data);
  void missionManagerMakePlan(const Json::Value &, Json::Value &ack_data);

  void missionManagerGetPathEstimateTime(const Json::Value &,
                                         Json::Value &ack_data);

  void missionManagerAdditionalClean(const Json::Value &rev_data,
                                     Json::Value &ack_data);

  void missionManagerClearClean(const Json::Value &rev_data,
                                Json::Value &ack_data);

  void missionManagerCallback(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
          request,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
          response);

  void publishPath(
      const std::vector<Pose2d> &path,
      const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pub);
  void publishPoseArray(
      const std::vector<Pose2d> &plan_path,
      const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr &pub);

  void publishCmdVel(const Velocity &velocity);
  void publishCarrot(const Pose2d &carrot_pose);
  void publishRemainPath(double path_length);

  // 设置和修改导航状态与错误码，保证线程安全
  void setNaviState(const NavigationState &state);
  void setErrorCode(const NavigationErrorCode &code);
  void setPathType(MISSIONTYPE type);
  void setNaviType(const Json::Value &rev_data);
  NavigationState getNaviState();
  NavigationErrorCode getErrorCode();

  // 获取和设置里程与定位是否超时的数据
  void updateOdomTime();
  void updateLocalizationTime();
  std::chrono::duration<double> calcOdomTimeDiff();
  std::chrono::duration<double> calcLocalizationTimeDiff();

  //   void calcLogicSpeedLevel();
  void brakeRobot(bool repeat_flag = true);
  //   void ctrlLightWithPathDirection();
};

}  // namespace CVTE_BABOT

POINT_CLOUD_REGISTER_POINT_STRUCT(
    CVTE_BABOT::MapGridCostPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, path_cost, path_cost)(
        float, goal_cost, goal_cost)(float, occ_cost,
                                     occ_cost)(float, total_cost, total_cost))

#endif