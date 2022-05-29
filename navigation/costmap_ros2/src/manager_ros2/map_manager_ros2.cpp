#include "manager_ros2/map_manager_ros2.hpp"

#include <angles/angles.h>

#include "common/costmap_params_ros2.hpp"
#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include "costmap_cloud.hpp"
#include "costmap_range_data.hpp"
#include "debug_log.hpp"
#include "footprint_utils.hpp"
#include "manager_ros2/data_manager_ros2.hpp"
#include "obstacle_map.hpp"
#include "visualizer_ros2/costmap_visualizer_ros2.hpp"
#include "world_map_data.hpp"

namespace CVTE_BABOT {
float epsilon = 0.0001;
using namespace std::chrono_literals;
CostmapArchAdapter::CostmapArchAdapter(const std::string &name)
    : node_name_(name),
      extra_slow_down_area_(0.0, 0.0),
      extra_stop_area_(0.0, 0.0) {
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>(name, options);
  // timer_node_ = std::make_shared<rclcpp::Node>(name + "_timer");

  costmap_params_ros2_ = std::make_shared<CostmapParamsRos2>(node_);

  ptr_costmap_params_ = std::make_shared<CostmapParameters>();
  costmap_params_ros2_->setCostmapParameters(ptr_costmap_params_);

  CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params_);

  ptr_data_manager_ros2_ =
      std::make_shared<DataManagerRos2>(node_, costmap_params_ros2_);

  ptr_costmap_visualizer_ros2_ = std::make_shared<CostmapVisualizerRos2>(node_);

  std::shared_ptr<WorldmapData> ptr_wm_data;
  CostmapMediator::getPtrInstance()
      ->registerUpdateFunc<std::shared_ptr<WorldmapData>>("static_map",
                                                          ptr_wm_data);
  cosmap_thread_ = nullptr;
  pub_cosmap_thread_ = nullptr;
  costmap_id_ = 0;

  // std::shared_ptr<const Costmap2d> ptr_costmap;
  // CostmapMediator::getPtrInstance()
  //     ->registerUpdateFunc<std::shared_ptr<const Costmap2d>>("costmap",
  //                                                            ptr_costmap);
}

void CostmapArchAdapter::updateParameter() {
  // enable or disable printing debug log
  bool debug_mode;
  node_->get_parameter_or(std::string("debug_mode"), debug_mode, false);
  LOG(INFO) << debug_mode;
  if (debug_mode) {
    ENABLE_DEBUG_LOG
  } else {
    DISABLE_DEBUG_LOG
  }

  node_->get_parameter_or(std::string("update_frequency"), update_frequency_,
                          5.0);
  node_->get_parameter_or(std::string("reset_costmap_time_threshhold"),
                          ui_reset_costmap_time_threshhold_, (unsigned int)10);
  node_->get_parameter_or(
      std::string("footprint"), str_footprint_,
      std::string(
          "[[-0.2, 0.35], [0.70, 0.35], [0.20, -0.35], [0.70, -0.35]]"));
  node_->get_parameter_or(std::string("footprint_padding"),
                          d_footprint_padding_, 0.01);

  node_->get_parameter_or("map_path", map_path_, std::string(""));
  node_->get_parameter("init_static_map", b_init_static_map_);
  LOG(ERROR) << b_init_static_map_;
  b_init_static_map_ = false;
  node_->get_parameter_or("rolling_window", rolling_window_, true);
  node_->get_parameter_or("mark_unknown_space", b_mark_unknown_space_, true);
  ptr_costmap_params_->setParam("mark_unknown_space", b_mark_unknown_space_);
  node_->get_parameter_or("width", width_, 20.0);
  node_->get_parameter_or("height", height_, 20.0);
  node_->get_parameter_or("resolution", resolution_, 0.05);

  node_->get_parameter_or("extra_slow_down_area_size_x",
                          d_extra_slow_down_area_size_x_, 0.3);
  node_->get_parameter_or("extra_slow_down_area_size_y",
                          d_extra_slow_down_area_size_y_, 0.2);
  node_->get_parameter_or("extra_slow_down_area_threshold_x",
                          d_extra_slow_down_area_threshold_x_, 0.25);
  node_->get_parameter_or("extra_slow_down_area_threshold_w",
                          d_extra_slow_down_area_threshold_w_, 0.2);

  node_->get_parameter_or("extra_stop_area_size_x", d_extra_stop_area_size_x_,
                          0.3);
  node_->get_parameter_or("extra_stop_area_size_y", d_extra_stop_area_size_y_,
                          0.2);
  node_->get_parameter_or("extra_stop_area_threshold_x",
                          d_extra_stop_area_threshold_x_, 0.25);
  node_->get_parameter_or("extra_stop_area_threshold_w",
                          d_extra_stop_area_threshold_w_, 0.2);
  node_->get_parameter_or("extra_area_after_stop_size_x",
                          d_extra_area_after_stop_size_x_, 0.7);
  ptr_costmap_params_->setParam("extra_area_after_stop_size_x",
                                d_extra_area_after_stop_size_x_);

  node_->get_parameter_or("pose_topic", pose_topic_,
                          std::string("fusion_pose"));

  node_->get_parameter_or("publish_costmap", b_publish_costmap_, true);
  node_->get_parameter_or("publish_voxel", b_publish_voxel_, false);
  ptr_costmap_params_->setParam("publish_voxel", b_publish_voxel_);
  node_->get_parameter_or("publish_clearing_points", b_publish_clearing_points_,
                          false);
  ptr_costmap_params_->setParam("publish_clearing_points",
                                b_publish_clearing_points_);
  node_->get_parameter_or("cleraing_cloud_name", s_cleraing_cloud_name_,
                          std::string("clearing_points"));
  ptr_costmap_params_->setParam("cleraing_cloud_name", s_cleraing_cloud_name_);

  node_->get_parameter_or("use_navigation_localization",
                          b_use_navigation_localization_, false);
  node_->get_parameter_or("use_navigation_cmd_vel", b_use_navigation_cmd_vel_,
                          false);

  bool static_enable_layer = false;
  node_->get_parameter_or("static_layer.enabled", static_enable_layer, false);
  if (static_enable_layer) {
    costmap_params_ros2_->loadStaticLayerParams();
  }

  bool obstacle_enable_layer = false;
  node_->get_parameter_or("obstacle_layer.enabled", obstacle_enable_layer,
                          false);
  if (obstacle_enable_layer) {
    costmap_params_ros2_->loadObstacleLayerParams();
  }

  bool sonar_enable_layer = false;
  node_->get_parameter_or("sonar_layer.enabled", sonar_enable_layer, false);
  if (sonar_enable_layer) {
    costmap_params_ros2_->loadSonarLayerParams();
  }

  bool range_sensor_enable_layer2 = false;
  node_->get_parameter_or("range_layer.enabled", range_sensor_enable_layer2,
                          false);
  if (range_sensor_enable_layer2) {
    costmap_params_ros2_->loadRangeSensorLayer2Params();
  }

  bool collision_enable_layer = false;
  node_->get_parameter_or("collision_layer.enabled", collision_enable_layer,
                          false);
  if (collision_enable_layer) {
    costmap_params_ros2_->loadCollisionLayerParams();
  }

  bool infaltion_enable_layer = false;
  node_->get_parameter_or("inflation_layer.enabled", infaltion_enable_layer,
                          false);

  if (infaltion_enable_layer) {
    costmap_params_ros2_->loadInflationLayerParams();
  }

  bool clean_area_enable = false;
  node_->get_parameter_or("clean_area_enable", clean_area_enable, false);
  // clean_area_enable_ = clean_area_enable;

  ptr_data_manager_ros2_->updateParameter();
}

bool CostmapArchAdapter::systemInit() {
  CostmapMediator::getPtrInstance()->setResetStaticMapFlag(false);

  ptr_layered_costmap_ =
      std::make_shared<LayeredCostmap>(rolling_window_, b_mark_unknown_space_);

  // if (clean_area_enable_) {
  //   LOG(INFO) << "set clean area enable.";
  //   ptr_layered_costmap_->setSlopeArea();
  // }

  if (rolling_window_) {
    double map_width_meters = width_, map_height_meters = height_,
           origin_x = 0.0, origin_y = 0.0;
    if (!ptr_layered_costmap_->isSizeLocked()) {
      ptr_layered_costmap_->resizeMap(
          (unsigned int)(map_width_meters / resolution_),
          (unsigned int)(map_height_meters / resolution_), resolution_,
          origin_x, origin_y);
    }
  }

  CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap_);
  ptr_costmap_ = std::make_shared<Costmap2d>(
      *(CostmapMediator::getPtrInstance()->getCostmap()));
  upateCostmapId();

  // 必须在构造costmap之前，不然footprint未设置costmap构造参数有问题
  std::vector<WorldmapPoint> v_new_footprint;
  if (makeFootprintFromString(str_footprint_, v_new_footprint)) {
    ptr_costmap_params_->setParam("footprint", v_new_footprint);
    v_padded_footprint_ = v_new_footprint;
    padFootprint(d_footprint_padding_, v_padded_footprint_);
    setpaddedRobotFootprint(v_padded_footprint_);
    ptr_costmap_params_->setParam("footprint_padded", v_padded_footprint_);
  } else {
    LOG(ERROR) << "Invalid footprint string" << std::endl;
  }

  auto builder = std::make_shared<Costmap2dBuilder>(ptr_layered_costmap_);
  auto director = std::make_shared<Costmap2dDirector>(builder);

  director->buildCostmap(costmap_params_ros2_->layer_param_);
  LOG(INFO) << "BuildCostmap finished.";

  if (b_init_static_map_) {
    if (!resetStaticMap(map_path_)) {
      return false;
    }
  } else {
    CostmapMediator::getPtrInstance()->setResetStaticMapFlag(false);
  }

  // 初始化processor_arch_adapter
  if (!ptr_data_manager_ros2_->systemInit()) {
    return false;
  }

  // 初始化costmap_visualizer_ros2
  ptr_costmap_visualizer_ros2_->systemInit();

  std::vector<double> sensor_tf(3, 0.0);
  CostmapMediator::getPtrInstance()->getParam("laser.sensor_tf", sensor_tf,
                                              sensor_tf);

  // ptr_speed_pub_ =
  //     node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel");

  // ptr_repulsion_pub_ =
  //     node_->create_publisher<geometry_msgs::msg::PointStamped>("repulsion");

  ptr_reset_costmap_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      std::string("reset_costmap"), rclcpp::QoS(10).best_effort(),
      [this](const std::shared_ptr<std_msgs::msg::Int8> ptr_message) {
        if (ptr_message->data == 1) {
          resetStaticMap(map_path_);
        } else if (ptr_message->data == 2) {
          CostmapMediator::getPtrInstance()->setResetCostmapFlag(true);
        } else if (ptr_message->data == 3) {
          startCostmapTimer();
        } else if (ptr_message->data == 4) {
          stopCostmapTimer();
        } else if (ptr_message->data == 5) {
          // if (avoidance_timer_ == nullptr) {
          //   startAvoidanceTimer();
          // }
        } else if (ptr_message->data == 6) {
          // avoidance_timer_.reset();
        }
      });

  LOG(INFO) << "systemInit finish.";

  return true;
}

void CostmapArchAdapter::activeLayer(LayerName layer) {
  ptr_layered_costmap_->activeLayer(layer);
}

void CostmapArchAdapter::deactiveLayer(LayerName layer) {
  ptr_layered_costmap_->deactiveLayer(layer);
}

void CostmapArchAdapter::resetLayer(LayerName layer) {
  ptr_layered_costmap_->resetLayer(layer);
}

void CostmapArchAdapter::pubCostmapTimeCallback() {
  while (rclcpp::ok() && !stop_thread_) {
    // 6.可视化
    ptr_costmap_visualizer_ros2_->visFootprint(robot_current_pose_,
                                               v_padded_footprint_);

    if (b_publish_costmap_) {
      std::unique_lock<std::recursive_mutex> lock(*ptr_costmap_->getMutx());
      ptr_costmap_visualizer_ros2_->visCostmap(ptr_costmap_);
    }

    if (b_publish_clearing_points_) {
      auto ptr_cloud = std::make_shared<CostmapCloud>();
      if (CostmapMediator::getPtrInstance()->getData(s_cleraing_cloud_name_,
                                                     ptr_cloud)) {
        ptr_costmap_visualizer_ros2_->visClearingPoints(ptr_cloud);
      }
    }

    if (b_publish_voxel_) {
      auto ptr_cloud = std::make_shared<CostmapCloud>();
      if (CostmapMediator::getPtrInstance()->getData("voxel_points",
                                                     ptr_cloud)) {
        ptr_costmap_visualizer_ros2_->visVoxelGrid(ptr_cloud);
      }
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(200)));
  }
}

void CostmapArchAdapter::costmapTimerCallback() {
  while (rclcpp::ok() && !stop_thread_) {
    // 1.获取位姿
    if (run_thread_) {
      WorldmapPose robot_current_pose;
      {
        std::lock_guard<std::mutex> lock(localization_mutex_);
        robot_current_pose = robot_current_pose_;
      }

      // 2.定时清除costmap障碍层
      if (ui_reset_costmap_count_ < ui_reset_costmap_time_threshhold_) {
        ui_reset_costmap_count_++;
      } else {
        ui_reset_costmap_count_ = 0;
        CostmapMediator::getPtrInstance()->setResetCostmapFlag(true);
      }

      // 3.更新costmap
      if (!ptr_layered_costmap_->updateMap(robot_current_pose)) {
        // LOG(INFO) << "updateMap failed. " << std::endl;
      }

      // 4.还原清除costmap的标志,标志不会自动清除
      CostmapMediator::getPtrInstance()->setResetCostmapFlag(false);

      // 5.拷贝存储地图
      ptr_costmap_ = std::make_shared<Costmap2d>(
          *(CostmapMediator::getPtrInstance()->getCostmap()));
      upateCostmapId();
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(100)));
  }
}

void CostmapArchAdapter::localizationDataCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        localization_msg) {
  // if this function called by multic thread in the future, can't declare
  // variables as static!
  std::lock_guard<std::mutex> lock(localization_mutex_);

  robot_current_pose_.d_x = localization_msg->pose.pose.position.x;
  robot_current_pose_.d_y = localization_msg->pose.pose.position.y;

  AngleCalculate::Quaternion q;
  q.x = localization_msg->pose.pose.orientation.x;
  q.y = localization_msg->pose.pose.orientation.y;
  q.z = localization_msg->pose.pose.orientation.z;
  q.w = localization_msg->pose.pose.orientation.w;
  robot_current_pose_.d_yaw = AngleCalculate::quaternionToYaw(q);

  TimeType time = tf2_ros::timeToSec(localization_msg->header.stamp);

  CostmapMediator::getPtrInstance()->setPose({robot_current_pose_.d_x,
                                              robot_current_pose_.d_y,
                                              robot_current_pose_.d_yaw, time});
}

bool CostmapArchAdapter::resetStaticMap(const std::string &map_path) {
  // TODO (华勃) : 多线程调用保证安全
  auto ptr_mapdata = std::make_shared<WorldmapData>();

  if (ptr_mapdata->readMapFromYaml(map_path)) {
    LOG(INFO) << "read map yaml file ok ";

    // 为了保证线程安全，在进行静态地图更新前将地图更新线程停掉
    // costmap_timer_.reset();
    run_thread_ = false;
    LOG(INFO) << "reset costmap timer";

    CostmapMediator::getPtrInstance()->updateData("static_map", ptr_mapdata);
    CostmapMediator::getPtrInstance()->setResetStaticMapFlag(true);

    WorldmapPose robot_current_pose;
    {
      std::lock_guard<std::mutex> lock(localization_mutex_);
      robot_current_pose = robot_current_pose_;
    }

    if (ptr_layered_costmap_->updateMap(robot_current_pose)) {
      // 重置地图更新后需要及时更新输出的代价地图
      ptr_costmap_ = std::make_shared<Costmap2d>(
          *(CostmapMediator::getPtrInstance()->getCostmap()));
      upateCostmapId();
      LOG(INFO) << "restart costmap timer ok";
      run_thread_ = true;
      return true;
    } else {
      LOG(ERROR) << "updateMap failed. " << std::endl;
      return false;
    }
  } else {
    LOG(ERROR) << "failed to load map." << std::endl;
    CostmapMediator::getPtrInstance()->setResetStaticMapFlag(false);

    return false;
  }
}

bool CostmapArchAdapter::existInitStaticMap(std::string &map_path) {
  if (b_init_static_map_) {
    map_path = map_path_;
    return true;
  }

  return false;
}

void CostmapArchAdapter::rangeTopicReceiveSet(int dir, bool enable) {
  if (enable) {
    ptr_data_manager_ros2_->startDirRangeIntput(dir);
  } else {
    ptr_data_manager_ros2_->stopDirRangeIntput(dir);
  }
}

void CostmapArchAdapter::setpaddedRobotFootprint(
    const std::vector<WorldmapPoint> &points) {
  ptr_layered_costmap_->setFootprint(points);
}

void CostmapArchAdapter::startCostmapTimer() {
  LOG(INFO) << "startCostmapTimer begin. ";

  if (nullptr == localization_sub_ && !b_use_navigation_localization_) {
    localization_sub_ = node_->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, rclcpp::QoS(10).best_effort(),
        std::bind(&CostmapArchAdapter::localizationDataCallback, this,
                  std::placeholders::_1));
  }
  stop_thread_ = false;
  run_thread_ = true;
  if (cosmap_thread_ == nullptr) {
    cosmap_thread_ = new std::thread(
        std::bind(&CostmapArchAdapter::costmapTimerCallback, this));
  }
  if (pub_cosmap_thread_ == nullptr) {
    pub_cosmap_thread_ = new std::thread(
        std::bind(&CostmapArchAdapter::pubCostmapTimeCallback, this));
  }
  ptr_data_manager_ros2_->startProcessorTimer();
  LOG(INFO) << "startCostmapTimer finished";
}

void CostmapArchAdapter::stopCostmapTimer() {
  LOG(INFO) << "stopCostmapTimer begin";

  // costmap_timer_.reset();

  // avoidance_timer_.reset();
  stop_thread_ = true;
  if (cosmap_thread_->joinable()) {
    cosmap_thread_->join();
  }
  if (pub_cosmap_thread_->joinable()) {
    pub_cosmap_thread_->join();
  }
  free(cosmap_thread_);
  free(pub_cosmap_thread_);
  cosmap_thread_ = nullptr;
  pub_cosmap_thread_ = nullptr;

  CostmapMediator::getPtrInstance()->setResetCostmapFlag(true);

  // reset after localizationDataCallback executing finished.
  {
    std::lock_guard<std::mutex> lock(localization_mutex_);
    localization_sub_.reset();
  }

  // reset after cmdVelCallback executing finished.
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    cmd_vel_sub_.reset();
  }

  ptr_data_manager_ros2_->stopProcessorTimer();
  LOG(INFO) << "stopCostmapTimer finished";
}

void CostmapArchAdapter::spin() {
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_);
  // exec.add_node(timer_node_);
  exec.spin();
  exec.remove_node(node_);
  // exec.remove_node(timer_node_);
}
}  // namespace CVTE_BABOT
