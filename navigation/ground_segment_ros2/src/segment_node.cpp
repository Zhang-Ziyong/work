#include "log.hpp"
#include "point_cloud_tools.hpp"
#include "segment.hpp"

namespace CVTE_BABOT {

const double VISUALIZATION_RADIUS = 1.000;
const double VISUALIZATION_RESOLUTION = 0.100;
const double EPSILON = 0.001;

bool pointCmp(const VPoint &a, const VPoint &b) {
  return a.z < b.z;
}

// 判断vector的某一元素是否存在
bool isElementInVector(std::vector<int> v, int element) {
  std::vector<int>::iterator it;
  it = find(v.begin(), v.end(), element);
  if (it != v.end()) {
    return true;
  } else {
    return false;
  }
}

GroundSegment::GroundSegment()
    : ptr_no_ground_(new pcl::PointCloud<VPoint>),
      ptr_ground_(new pcl::PointCloud<VPoint>),
      ptr_not_ground_sum_(new pcl::PointCloud<VPoint>),
      ptr_ground_sum_(new pcl::PointCloud<VPoint>),
      ptr_cloud_raw_(new pcl::PointCloud<VPoint>),
      ptr_cloud_processed_(new pcl::PointCloud<VPoint>) {
  ptr_not_ground_sum_->height = 1;
  ptr_ground_sum_->height = 1;

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>("ground_segment", options);

  // 创建node,clock,point_cluster后才能获取参数
  updateParameter();

  static_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "static_points", rclcpp::QoS(10).best_effort());
  obstacle_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "all_obstacle_points", rclcpp::QoS(10).best_effort());
  all_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "all_points", rclcpp::QoS(10).best_effort());
  curb_point_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "curb_points", rclcpp::QoS(10).best_effort());
  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      s_topic_name_, rclcpp::QoS(10).best_effort(),
      std::bind(&GroundSegment::pointCloudCallback, this,
                std::placeholders::_1));

  scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::QoS(5).best_effort());
  voxel_filter_no_ground_ = std::make_shared<VoxelGridFilter>(
      region_interest_, region_interest_, sensor_height_ + 1.0);
  voxel_filter_ground_ = std::make_shared<VoxelGridFilter>(
      region_interest_, region_interest_, sensor_height_ + 1.0);
  int grid_length = 2 * region_interest_ / grid_resolution_;
  grid_pts_.resize(grid_length);
  for (int j = 0; j < grid_length; ++j) { grid_pts_[j].resize(grid_length); }

  // 初始化激光发布
  laser_scan_msg_.header.frame_id =
      "laser";  // assume laser has the same frame as the robot

  laser_scan_msg_.angle_min = -M_PI;
  laser_scan_msg_.angle_max = M_PI;
  laser_scan_msg_.angle_increment = 1.0f / 180 * M_PI;
  laser_scan_msg_.time_increment = 0;

  laser_scan_msg_.scan_time = 0.1;
  laser_scan_msg_.range_min = 0.3;
  laser_scan_msg_.range_max = 50;

  int range_size =
      std::ceil((laser_scan_msg_.angle_max - laser_scan_msg_.angle_min) /
                laser_scan_msg_.angle_increment);
  laser_scan_msg_.ranges.assign(range_size, laser_scan_msg_.range_max);

  int size = VISUALIZATION_RADIUS / VISUALIZATION_RESOLUTION;
  LOG(INFO) << "the visualization map's size: " << VISUALIZATION_RADIUS << " * "
            << size << " m*m.";

  std::vector<uint32_t> column(2 * size, 0);
  vv_point_cells_.resize(2 * size, column);
  LOG(INFO) << "printCoverCellsInMap";
  printCoverCellsInMap(vv_point_cells_);
}

void GroundSegment::printCoverCellsInMap(
    const std::vector<std::vector<uint32_t>> &vvui_point_cells) {
  /* 坐标系
        x     ix
        ^
        |
        |
  y <--------- 3
        |      2
        |      1
  iy    |  2 1 0
  */
  // 通过直接调用glog内部的打印流,把内部封裝的自动换行去掉.
  // 因为glog实例化的对象设置成了无法拷贝,所以使用右值引用
  auto &&log = COMPACT_GOOGLE_LOG_INFO;

  // 先换行
  log.stream() << std::endl;
  for (int ix = vvui_point_cells.size() - 1; ix >= 0; ix--) {
    for (int iy = vvui_point_cells[ix].size() - 1; iy >= 0; iy--) {
      uint32_t value = vvui_point_cells[ix][iy];

      if ((static_cast<unsigned int>(ix) == vvui_point_cells.size() / 2) &&
          (static_cast<unsigned int>(iy) == vvui_point_cells[ix].size() / 2)) {
        log.stream() << " - ";
      } else if (value == 0) {
        log.stream() << " 0 ";
      } else {
        int count = ((value + 10) / 10);
        if (count < 10) {
          log.stream() << " " << std::to_string(count) << " ";
        } else {
          log.stream() << " * ";
        }
      }
    }
    log.stream() << std::endl;
  }
}

void GroundSegment::updateParameter() {
  node_->get_parameter_or("print_log", b_print_log_, false);

  node_->get_parameter_or("topic_name", s_topic_name_,
                          std::string("/livox/lidar"));

  node_->get_parameter_or("region_intesest", region_interest_, 7);
  node_->get_parameter_or("region_filter_threshold", d_region_filter_threshold_,
                          0.95);
  node_->get_parameter_or("sensor_height", sensor_height_, 0.85);

  node_->get_parameter_or("grid_resolution", grid_resolution_, 0.2);
  node_->get_parameter_or("th_grid_het", th_grid_het_, 0.2);
  node_->get_parameter_or("curb_z", curb_z_, 0.1);
  node_->get_parameter_or("cloud_sum", cloud_sum_, 8);
  node_->get_parameter_or("curb_sum", curb_sum_, true);

  node_->get_parameter_or("search_k", i_search_k_, 30);

  node_->get_parameter_or("set_ground_zero", b_set_ground_zero_, true);

  node_->get_parameter_or("downsample", b_downsample_, true);

  node_->get_parameter_or("remove_outlier", b_remove_outlier_, false);
  node_->get_parameter_or("min_neighbors_in_radius", i_min_neighbors_in_radius_,
                          10);
  node_->get_parameter_or("remove_outlier_radius", d_remove_outlier_radius_,
                          0.3);
}

void GroundSegment::resetMap(
    std::vector<std::vector<uint32_t>> &vvui_point_cells) {
  for (auto &vui_point_cells : vvui_point_cells) {
    for (auto &ui_point_cells : vui_point_cells) { ui_point_cells = 0; }
  }
}

void GroundSegment::setPointToMap(
    const double &x, const double &y,
    std::vector<std::vector<uint32_t>> &vvui_point_cells) {
  double threshold = VISUALIZATION_RADIUS - EPSILON;

  if (x < -threshold || threshold < x || y < -threshold || threshold < y) {
    return;
  }

  int ix = (x + threshold) / VISUALIZATION_RESOLUTION;
  int iy = (y + threshold) / VISUALIZATION_RESOLUTION;

  vvui_point_cells[ix][iy]++;
}

void GroundSegment::transformAndFilterCloud(
    const VPointsPtr &ptr_cloud_raw, const VPointsPtr &ptr_cloud_processed) {
  const double distance_threshold_square = 1.0 * 1.0;
  double outlier_radius_square =
      d_remove_outlier_radius_ * d_remove_outlier_radius_;
  for (const auto &raw_point : ptr_cloud_raw->points) {
    // 2.只获取感兴趣的区域的点
    // -0.001防止等于x或y的值刚好等于region_interest_导致之后越界
    if ((fabs(raw_point.x) > region_interest_ - 0.001) ||
        (fabs(raw_point.y) > region_interest_ - 0.001) ||
        (fabs(raw_point.z) > 1.0 - 0.001)) {
      continue;
    }
    ptr_cloud_processed->points.push_back(raw_point);

    // double range_square = raw_point.x * raw_point.x + raw_point.y *
    // raw_point.y;
    // // 把距离小于阀值的点单独处理
    // if (range_square < distance_threshold_square &&
    //     range_square > outlier_radius_square) {
    //   // 不在四根柱子遮挡范围内的才加入处理
    //   double angle = atan2(raw_point.y, raw_point.x);
    //   // 模拟四根杆-142.5到-127.5, -52.5到-37.5，37.5到52.5, 127.5到142.5
    //   bool angle_in_blind_area =
    //       ((-M_PI * 14.25 / 18.0) < angle && (angle < -M_PI * 12.75 / 18.0))
    //       ||
    //       ((-M_PI * 5.25 / 18.0) < angle && (angle < -M_PI * 3.75 / 18.0)) ||
    //       ((M_PI * 3.75 / 18.0 < angle) && (angle < M_PI * 5.25 / 18.0)) ||
    //       ((M_PI * 12.75 / 18.0 < angle) && (angle < M_PI * 14.25 / 18.0));
    //   if (!angle_in_blind_area) {
    //     ptr_cloud_processed->points.push_back(raw_point);
    //   }
    // } else {
    //   ptr_cloud_processed->points.push_back(raw_point);
    // }
  }

  if (b_print_log_) {
    static int print_count = 0;
    if (print_count > 10) {
      print_count = 0;
      resetMap(vv_point_cells_);
      for (const auto &point : ptr_cloud_processed->points) {
        setPointToMap(point.x, point.y, vv_point_cells_);
      }
      LOG(INFO) << "print point cloud";
      printCoverCellsInMap(vv_point_cells_);
    } else {
      print_count++;
    }
  }
}

void GroundSegment::updateScanMsg(const double &x, const double &y,
                                  sensor_msgs::msg::LaserScan &laser_scan_msg) {
  double angle = std::atan2(y, x);
  if (angle < laser_scan_msg.angle_min || laser_scan_msg.angle_max < angle) {
    return;
  }

  float range = std::sqrt(x * x + y * y);
  int index =
      (angle - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment;

  laser_scan_msg.ranges[index] = std::min(laser_scan_msg.ranges[index], range);
}

void GroundSegment::computeAndPubLaserScan(
    const VPointsPtr &ptr_obstacle_cloud,
    const builtin_interfaces::msg::Time &time_stamp) {
  // convert point to scan
  for (const auto &point : ptr_obstacle_cloud->points) {
    updateScanMsg(point.x, point.y, laser_scan_msg_);
  }

  laser_scan_msg_.header.stamp = time_stamp;
  scan_pub_->publish(laser_scan_msg_);

  if (b_print_log_) {
    static int print_count = 0;
    if (print_count > 10) {
      print_count = 0;
      resetMap(vv_point_cells_);

      for (size_t i = 0; i < laser_scan_msg_.ranges.size(); i++) {
        double angle =
            laser_scan_msg_.angle_min + laser_scan_msg_.angle_increment * i;

        double x = laser_scan_msg_.ranges[i] * cos(angle);
        double y = laser_scan_msg_.ranges[i] * sin(angle);
        setPointToMap(x, y, vv_point_cells_);
      }
      LOG(INFO) << "print scan.";
      printCoverCellsInMap(vv_point_cells_);
    } else {
      print_count++;
    }
  }

  std::fill(laser_scan_msg_.ranges.begin(), laser_scan_msg_.ranges.end(),
            laser_scan_msg_.range_max);
}

/*
    pointcloud callback function.
    1.read pointcloud and build the characteristics of points
    2.extract ground seeds and ground plane fitter
    3.output is ground points and off-ground points
*/
void GroundSegment::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud_msg) {
  // ptr_cloud_raw_->clear();
  ptr_cloud_raw_.reset(new pcl::PointCloud<VPoint>);

  // 2. get interest area
  // ptr_cloud_processed_->clear();
  VPointsPtr laser_cloud_in(new pcl::PointCloud<VPoint>);
  std::vector<int> indices;
  pcl::fromROSMsg(*in_cloud_msg, *ptr_cloud_raw_);
  pcl::removeNaNFromPointCloud(*ptr_cloud_raw_, *laser_cloud_in, indices);
  ptr_cloud_processed_.reset(new pcl::PointCloud<VPoint>);
  // ptr_cloud_processed_->reserve(ptr_cloud_raw_->points.size());
  transformAndFilterCloud(laser_cloud_in, ptr_cloud_processed_);

  // 3. grid filtering
  ptr_no_ground_.reset(new pcl::PointCloud<VPoint>);
  // ptr_no_ground_->clear();
  // ptr_no_ground_->reserve(ptr_cloud_processed_->size());

  // ptr_ground_->clear();
  ptr_ground_.reset(new pcl::PointCloud<VPoint>);
  VPointsPtr ptr_static_cloud(new pcl::PointCloud<VPoint>);
  // ptr_ground_->reserve(ptr_cloud_processed_->size());
  gridMapFilter(ptr_cloud_processed_, ptr_no_ground_, ptr_ground_,
                ptr_static_cloud);

  if (b_downsample_ && ptr_no_ground_->size() != 0) {
    voxel_filter_no_ground_->setInputCloud(ptr_no_ground_);
    voxel_filter_no_ground_->setLeafSize(0.20, 0.20, 0.10);
    voxel_filter_no_ground_->filter(*ptr_no_ground_);
  }

  computeAndPubLaserScan(ptr_no_ground_, in_cloud_msg->header.stamp);

  sensor_msgs::msg::PointCloud2 static_cloud_msg;
  pcl::toROSMsg(*ptr_static_cloud, static_cloud_msg);
  static_cloud_msg.header.stamp = in_cloud_msg->header.stamp;
  static_cloud_msg.header.frame_id = "laser";
  static_cloud_pub_->publish(static_cloud_msg);
  VPointsPtr all_points(new pcl::PointCloud<VPoint>);
  *all_points += *ptr_no_ground_;
  *all_points += *ptr_ground_;
  sensor_msgs::msg::PointCloud2 all_points_msg;
  pcl::toROSMsg(*all_points, all_points_msg);
  all_points_msg.header.stamp = in_cloud_msg->header.stamp;
  all_points_msg.header.frame_id = "laser";
  all_points_pub_->publish(all_points_msg);
  sensor_msgs::msg::PointCloud2 obstacle_cloud_msg;
  pcl::toROSMsg(*ptr_no_ground_, obstacle_cloud_msg);
  obstacle_cloud_msg.header.stamp = in_cloud_msg->header.stamp;
  obstacle_cloud_msg.header.frame_id = "laser";
  obstacle_cloud_pub_->publish(obstacle_cloud_msg);
}
/*
    The function to filter by grid height
    1. grid init
    2. fill the grid
    3. pre segment of pointcloud
*/
void GroundSegment::gridMapFilter(
    const pcl::PointCloud<VPoint>::ConstPtr ptr_cloud_in,
    const VPointsPtr ptr_no_ground, const VPointsPtr ptr_ground,
    const VPointsPtr ptr_static_cloud) {
  // grid map init
  int grid_length = 2 * region_interest_ / grid_resolution_;

  // fill the grid计算点位于哪个网格，填充
  for (auto &vp_grid : vvp_grid_filled_) {
    vp_grid->clear();
    vp_grid->shrink_to_fit();
  }
  vvp_grid_filled_.clear();
  vvp_grid_filled_.shrink_to_fit();

  for (const auto &point : ptr_cloud_in->points) {
    int GridI = (region_interest_ - point.x) / grid_resolution_;
    int GridJ = (region_interest_ - point.y) / grid_resolution_;
    assert(GridI >= 0);
    assert(GridJ >= 0);
    assert(GridI < grid_length);
    assert(GridJ < grid_length);

    auto vp_grid = &grid_pts_[GridI][GridJ];

    // 先前没有被填充点
    if (vp_grid->size() == 0) {
      vvp_grid_filled_.push_back(vp_grid);
    }

    vp_grid->push_back(point);
  }

  // calculate the grid height
  for (const auto &vp_grid : vvp_grid_filled_) {
    bool is_not_ground = false;
    float min_z = 1000.0;
    float max_z = -1000.0;

    for (auto it = vp_grid->begin(); it < vp_grid->end(); it += 2) {
      if (it->z > max_z) {
        max_z = it->z;
      }
      if (it->z < min_z) {
        min_z = it->z;
      }
      if (max_z - min_z > th_grid_het_) {
        is_not_ground = true;
        break;
      }
    }

    // float Height_dif = max->z - min->z;  // 网格中的高度差
    if (is_not_ground) {
      //根据高度差阈值和高度阈值，初步将点分为地面点和非地面点
      bool is_static_cloud = false;
      if (max_z + sensor_height_ < 0.3) {
        is_static_cloud = true;
      }
      for (const auto &point : *vp_grid) {
        ptr_no_ground->points.push_back(point);
        if (is_static_cloud) {
          ptr_static_cloud->points.push_back(point);
        }
      }
    } else {
      for (const auto &point : *vp_grid) {
        ptr_ground->points.push_back(point);
      }
    }
  }
}
}  // namespace CVTE_BABOT

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  CVTE_BABOT::initGoogleLog("ground_segment_ros2", "info");

  CVTE_BABOT::GroundSegment ground;
  rclcpp::spin(ground.getNodeHander());

  rclcpp::shutdown();
  return 0;
}
