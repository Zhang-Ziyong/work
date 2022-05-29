#include "glog/logging.h"
#include "ground.hpp"
#include "point_cloud_filter.hpp"
#include "point_cloud_tools.hpp"
#include "recon_cluster.hpp"

#include <pcl/segmentation/extract_clusters.h>

bool pointCmp(VPoint a, VPoint b) {
  return a.z < b.z;
}

bool pointSort(VPoint a, VPoint b) {
  return a.x < b.x;
}

double computBeta(const VPoint &pt) {
  double beta = atan2(pt.y, pt.x);
  double data;
  data = beta < 0. ? beta += 2 * M_PI : beta;
  return data;
}

/*
    points distance comparison function
*/
bool distCmp(VPoint a, VPoint b) {
  return sqrt(a.x * a.x + a.y * a.y) < sqrt(b.x * b.x + b.y * b.y);
}

// 判断vector的某一元素是否存在
bool isElementInVector(std::vector<int> v, int element) {
  static std::vector<int>::iterator it;
  it = find(v.begin(), v.end(), element);
  if (it != v.end()) {
    return true;
  } else {
    return false;
  }
}

void GroundPlaneFit::laserscanReceived(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  ptr_point_cloud_laser_->clear();
  ptr_point_cloud_laser_->reserve(scan_msg->ranges.size());

  static double range, point_x, point_y;

  for (unsigned int i = 0; i < scan_msg->ranges.size(); i++) {
    range = scan_msg->ranges[i];
    if (std::isnan(range) || range < 0.01 ||
        (!std::isfinite(range) && range > 0)) {
      range = scan_msg->range_max - 0.01;
    }

    // point in laser
    static float angle;
    angle = scan_msg->angle_min + i * scan_msg->angle_increment;

    point_x = range * cos(angle);
    point_y = range * sin(angle);

    static VPoint vp;
    vp.x = point_x;
    vp.y = point_y;

    if (fabs(vp.x) > 13.0 || fabs(vp.y) > 10.0) {
      continue;
    }

    // in x-y plane search the neighbor.
    vp.z = 0.0;
    ptr_point_cloud_laser_->push_back(vp);
  }
}

GroundPlaneFit::GroundPlaneFit()
    : ptr_no_ground_(new pcl::PointCloud<VPoint>),
      ptr_ground_(new pcl::PointCloud<VPoint>),
      ptr_not_ground_sum_(new pcl::PointCloud<VPoint>),
      ptr_ground_sum_(new pcl::PointCloud<VPoint>),
      ptr_curb_sum_(new pcl::PointCloud<VPoint>) {
  ptr_not_ground_sum_->height = 1;
  ptr_ground_sum_->height = 1;
  ptr_curb_sum_->height = 1;

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>("ground_fit", options);

  ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ptr_point_cluster_ =
      std::make_shared<CVTE_BABOT::PointCloudCluster>(ptr_clock_);

  // 创建node,clock,point_cluster后才能获取参数
  updateParameter();

  cloud_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>(
      "cloud_scan", rclcpp::QoS(10).best_effort());
  orbbec_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>(
      "cloud_scan_orb", rclcpp::QoS(10).best_effort());
  ground_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "ground_points", rclcpp::QoS(10).best_effort());
  boundmsgs_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "boxes", rclcpp::QoS(10).best_effort());
  curb_point_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "curb_points", rclcpp::QoS(10).best_effort());
  livox_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      s_topic_name_, rclcpp::QoS(10).best_effort(),
      std::bind(&GroundPlaneFit::livoxCallback, this, std::placeholders::_1));

  ptr_point_cloud_laser_ =
      pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);

  ptr_point_cloud_laser_copy_ =
      pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);

  if (b_fusion_2d_laser_) {
    rmw_qos_profile_t scan_qos = rmw_qos_profile_sensor_data;
    laser_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(10).best_effort(),
        std::bind(&GroundPlaneFit::laserscanReceived, this,
                  std::placeholders::_1),
        scan_qos);
  }

  cloud_count_ = 0;

  int grid_length = 2 * region_intesest_ / grid_resolution_;
  grid_pts_.resize(grid_length);
  for (int j = 0; j < grid_length; ++j) { grid_pts_[j].resize(grid_length); }

  // boost::interprocess::shared_memory_object::remove("shm");
  // m_managed_shm = boost::interprocess::managed_shared_memory(
  //     boost::interprocess::open_or_create, "shm",
  //     sizeof(struct shared_use_st) + 1024);
  // m_pData = m_managed_shm.find_or_construct<struct shared_use_st>("Data")();
  // m_pMtx =
  //     m_managed_shm.find_or_construct<boost::interprocess::interprocess_mutex>(
  //         "mtx")();

  // m_pMtx->lock();
  // m_pData->i_work = 0;
  // m_pMtx->unlock();
}

void GroundPlaneFit::updateParameter() {
  node_->get_parameter_or("print_log", b_print_log_, false);

  node_->get_parameter_or("topic_name", s_topic_name_,
                          std::string("/livox/lidar"));
  node_->get_parameter_or("fusion_2d_laser", b_fusion_2d_laser_, true);

  node_->get_parameter_or("region_intesest", region_intesest_, 7);
  node_->get_parameter_or("region_filter_x", d_region_filter_x_, 0.95);
  node_->get_parameter_or("sensor_height", sensor_height_, 0.85);
  node_->get_parameter_or("sensor_x", d_sensor_x_, -0.23);

  node_->get_parameter_or("sensor_pitch", d_sensor_pitch_, -0.385);
  node_->get_parameter_or("grid_resolution", grid_resolution_, 0.2);
  node_->get_parameter_or("th_grid_het", th_grid_het_, 0.4);
  node_->get_parameter_or("num_seg_x", num_seg_x_, 10);
  node_->get_parameter_or("num_seg_y", num_seg_y_, 1);
  node_->get_parameter_or("num_iter", num_iter_, 3);
  node_->get_parameter_or("num_lpr", num_lpr_, 20);
  node_->get_parameter_or("th_seeds", th_seeds_, 0.4);
  node_->get_parameter_or("th_dist", th_dist_, 0.2);
  node_->get_parameter_or("polar_angle", polar_angle_, 0.2);
  node_->get_parameter_or("search_radius", search_radius_, 0.3);
  node_->get_parameter_or("curb_z", curb_z_, 0.1);
  node_->get_parameter_or("cloud_sum", cloud_sum_, 8);
  node_->get_parameter_or("curb_sum", curb_sum_, true);

  node_->get_parameter_or("search_k", i_search_k_, 30);

  node_->get_parameter_or("pre_segment", b_pre_segment_, false);

  node_->get_parameter_or("segment_all_cloud", b_segment_all_cloud_, true);

  node_->get_parameter_or("ground_sum", b_ground_sum_, true);
  node_->get_parameter_or("set_ground_zero", b_set_ground_zero_, true);

  node_->get_parameter_or("min_neighbors_in_radius", i_min_neighbors_in_radius_,
                          5);
  node_->get_parameter_or("filter_search_radius", d_filter_search_radius_, 0.3);
  node_->get_parameter_or("max_cluster_size", i_max_cluster_size_, 1000);
  node_->get_parameter_or("remove_outlier", b_remove_outlier_, false);
  node_->get_parameter_or("downsample", b_downsample_, true);
  node_->get_parameter_or("remove_laser_plane", b_remove_laser_plane_, true);
  node_->get_parameter_or("remove_laser_plane_in_radius_",
                          b_remove_laser_plane_in_radius_, true);
  node_->get_parameter_or("remove_laser_plane_radius",
                          d_remove_laser_plane_radius_, 0.25);

  node_->get_parameter_or("curb_negative", b_curb_negative_, false);

  node_->get_parameter_or("distance_close", d_distance_close_, 1.5);
  node_->get_parameter_or("distance_far", d_distance_far_, 2.5);

  node_->get_parameter_or("up_height_close", d_up_height_close_, -0.05);
  node_->get_parameter_or("down_height_close", d_down_height_close_, 0.04);

  node_->get_parameter_or("up_height_far", d_up_height_far_, -0.05);
  node_->get_parameter_or("down_height_far", d_down_height_far_, -0.01);

  node_->get_parameter_or("down_count", i_down_count_, 2);
  node_->get_parameter_or("ground_count", i_ground_count_, 1);
  node_->get_parameter_or("drainage_count", i_drainage_count_, 1);
  node_->get_parameter_or("positive_threshold", d_positive_threshold_, 0.15);
  node_->get_parameter_or("curb_threshold", d_curb_threshold_, 0.08);
  node_->get_parameter_or("further_threshold", d_further_threshold_, -0.10);
  node_->get_parameter_or("further_count", i_further_count_, 20);

  node_->get_parameter_or("cluster_tolerance", d_cluster_tolerance_, 0.5);
  node_->get_parameter_or("increment_distance_radius",
                          d_increment_distance_radius_, 0.15);
  node_->get_parameter_or("clusters_radius", d_clusters_radius_, 0.25);
  node_->get_parameter_or("use_multi_radius", b_use_multi_radius_, true);
  node_->get_parameter_or("use_last_cluters", b_use_last_cluters_, true);
  node_->get_parameter_or("min_cluster_size", i_min_cluster_size_, 8);
  ptr_point_cluster_->updateParams(
      {d_cluster_tolerance_, d_increment_distance_radius_, d_clusters_radius_,
       b_use_multi_radius_, b_use_last_cluters_, i_min_cluster_size_});
}

void GroundPlaneFit::transformAndFilterCloud(
    const VPointsPtr &ptr_cloud_raw, const VPointsPtr &ptr_cloud_processed) {
  static double cos_yaw = cos(d_sensor_pitch_);
  static double sin_yaw = sin(d_sensor_pitch_);

  static VPoint point;

  for (size_t i = 0; i < ptr_cloud_raw->points.size(); i++) {
    // 1. Removes points with x, y, or z equal to NaN.
    if (!pcl_isfinite(ptr_cloud_raw->points[i].x) ||
        !pcl_isfinite(ptr_cloud_raw->points[i].y) ||
        !pcl_isfinite(ptr_cloud_raw->points[i].z)) {
      continue;
    }

    // 以传感器(0.00, 0.00, sensor_height_)为原点, 如果有斜装把角度较正过来
    point.x = cos_yaw * ptr_cloud_raw->points[i].x -
              sin_yaw * ptr_cloud_raw->points[i].z + 0.00;
    point.y = ptr_cloud_raw->points[i].y;
    point.z = sin_yaw * ptr_cloud_raw->points[i].x +
              cos_yaw * ptr_cloud_raw->points[i].z + sensor_height_;

    // 2. remove point insides the unreliable region
    if (fabs(point.x) < d_region_filter_x_ && fabs(point.y) < 0.3) {
      continue;
    }

    if (fabs(point.x) < region_intesest_ &&
        fabs(point.y) < region_intesest_ &&  // 只获取感兴趣的区域的点
        fabs(point.z) < sensor_height_ + 1.0) {
      point.intensity = ptr_cloud_raw->points[i].intensity;
      ptr_cloud_processed->points.push_back(point);
    }
  }
}

bool enforceCurvatureOrIntensitySimilarity(const pcl::PointNormal &point_a,
                                           const pcl::PointNormal &point_b) {
  const float *point_a_normal = point_a.normal;
  const float *point_b_normal = point_b.normal;
  if (fabs(point_a_normal[0] * point_b_normal[0] +
           point_a_normal[1] * point_b_normal[1] +
           point_a_normal[2] * point_b_normal[2]) < 0.05) {
    return true;
  }
  return false;
}

/*
    pointcloud callback function.
    1.read pointcloud and build the characteristics of points
    2.extract ground seeds and ground plane fitter
    3.output is ground points and off-ground points
*/
void GroundPlaneFit::livoxCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud_msg) {
  // static clock_t start, end;
  // start = clock();

  static VPointsPtr ptr_cloud_raw(new pcl::PointCloud<VPoint>);
  ptr_cloud_raw->clear();
  ptr_cloud_raw->reserve(in_cloud_msg->height * in_cloud_msg->width);

  // 1. Msg to raw pointcloud
  pcl::fromROSMsg(*in_cloud_msg,
                  *ptr_cloud_raw);  // ROS数据类型转换为pcl::PointCloud<T>

  // 2. get interest area
  static VPointsPtr ptr_cloud_processed(new pcl::PointCloud<VPoint>);

  ptr_cloud_processed->clear();
  ptr_cloud_processed->reserve(ptr_cloud_raw->points.size());
  transformAndFilterCloud(ptr_cloud_raw, ptr_cloud_processed);

  // 3. grid filtering
  ptr_no_ground_->clear();
  ptr_no_ground_->reserve(ptr_cloud_processed->size());

  ptr_ground_->clear();
  ptr_ground_->reserve(ptr_cloud_processed->size());
  gridMapFilter(ptr_cloud_processed, ptr_no_ground_, ptr_ground_);

  voxel_filter_.setInputCloud(ptr_ground_);
  voxel_filter_.setLeafSize(0.08, 0.08, 0.02);
  voxel_filter_.filter(*ptr_ground_);

  // if (b_remove_outlier_ && ptr_ground_->size() != 0) {
  //   outlier_removal_.setInputCloud(ptr_ground_);
  //   outlier_removal_.setMeanK(i_min_neighbors_in_radius_);
  //   outlier_removal_.setStddevMulThresh(d_filter_search_radius_);
  //   outlier_removal_.filter(*ptr_ground_);
  // }

  static VPointsPtr ptr_ground_sum(new pcl::PointCloud<VPoint>());
  ptr_ground_sum->clear();
  ptr_ground_sum->reserve(ptr_ground_->size());

  static VPointsPtr ptr_positive_curb(new pcl::PointCloud<VPoint>());
  ptr_positive_curb->clear();
  ptr_positive_curb->reserve(ptr_ground_->size());

  // 4.segment ground points, positive_curb and negative points.
  groundSegmentFurther(ptr_ground_, ptr_positive_curb, ptr_curb_sum_,
                       ptr_ground_sum);

  if (b_downsample_ && ptr_no_ground_->size() != 0) {
    uniform_sampling_filter_.setInputCloud(ptr_no_ground_);
    uniform_sampling_filter_.setRadiusSearch(0.06);
    uniform_sampling_filter_.filter(*ptr_no_ground_);
  }

  *ptr_not_ground_sum_ += *ptr_no_ground_;

  if (b_remove_outlier_ && ptr_positive_curb->size() != 0) {
    outlier_removal_.setInputCloud(ptr_positive_curb);
    outlier_removal_.setMeanK(i_min_neighbors_in_radius_);
    outlier_removal_.setStddevMulThresh(d_filter_search_radius_);
    outlier_removal_.filter(*ptr_positive_curb);
  }

  // regard as obstacle points.
  *ptr_not_ground_sum_ += *ptr_positive_curb;
  // for (size_t i = 0; i < ptr_ground_sum->points.size(); i++) {
  //   // if (!isElementInVector(curb_index, i)) {
  //   auto point = ptr_ground_sum->points[i];
  //   point.z = 0.0;
  //   ptr_ground_sum_->points.push_back(point);
  //   // }
  // }
  *ptr_ground_sum_ += *ptr_ground_sum;

  cloud_count_++;

  // Accumulate several period
  if (cloud_count_ == cloud_sum_) {
    // the negative points may contains the draingage points, detect and remove
    // it.
    if (b_remove_outlier_ && ptr_curb_sum_->size() != 0) {
      outlier_removal_.setInputCloud(ptr_curb_sum_);
      outlier_removal_.setMeanK(i_min_neighbors_in_radius_);
      outlier_removal_.setStddevMulThresh(d_filter_search_radius_);
      outlier_removal_.filter(*ptr_curb_sum_);
    }

    static VPointsPtr ptr_curb(new pcl::PointCloud<VPoint>);
    ptr_curb->clear();
    ptr_curb->reserve(2000);

    static VPointsPtr seg_cloud_part_x(new pcl::PointCloud<VPoint>);
    static VPointsPtr seg_cloud_part_y(new pcl::PointCloud<VPoint>);
    seg_cloud_part_x->reserve(200);
    seg_cloud_part_y->reserve(200);

    static float seg_step_x, seg_step_y;
    // to float
    seg_step_x = 1.0 * 4.0 / num_seg_x_;
    seg_step_y = 2.0 * 2.0 / num_seg_y_;
    static int seg_x, seg_y;

    // 直通滤波器,按照步长每次取出一定范围内的点云
    // 栅格化点云
    static pcl::PassThrough<VPoint> seg_pass;
    bool no_negative_grid = true;
    for (seg_y = 0; seg_y < num_seg_y_; seg_y++) {
      seg_cloud_part_y->clear();
      seg_pass.setInputCloud(ptr_curb_sum_);
      seg_pass.setFilterFieldName("y");
      seg_pass.setFilterLimits(-2.0 + seg_step_y * seg_y,
                               -2.0 + seg_step_y * (seg_y + 1));
      seg_pass.filter(*seg_cloud_part_y);

      // 遍历同y值上的点，看是否符合坑的模型。
      std::vector<int> v_ground_point_num;
      v_ground_point_num.resize(num_seg_x_, 0);

      for (seg_x = 0; seg_x < num_seg_x_; seg_x++) {
        seg_cloud_part_x->clear();
        seg_pass.setInputCloud(seg_cloud_part_y);
        seg_pass.setFilterFieldName("x");
        seg_pass.setFilterLimits(seg_step_x * seg_x, seg_step_x * (seg_x + 1));
        seg_pass.filter(*seg_cloud_part_x);

        int ground_count = 0;    //地面点的数量
        int negative_count = 0;  //负向点的数量

        for (size_t i = 0; i < seg_cloud_part_x->size(); i++) {
          auto searchPoint = seg_cloud_part_x->points[i];
          if (searchPoint.z < d_further_threshold_) {
            negative_count++;
          } else if (fabs(searchPoint.z) <= fabs(d_further_threshold_)) {
            ground_count++;
          }
        }

        // set the grid's type
        if (negative_count > i_further_count_) {
          v_ground_point_num[seg_x] = -negative_count;
        } else if (ground_count > 0) {
          v_ground_point_num[seg_x] = 1;
        }
      }

      // sort(v_ground_point_num.begin(), v_ground_point_num.end(), pointSort);

      int ground_grid_count = 0;
      int negative_grid_count = 0;
      int free_grid_count = 0;
      bool has_unpassable_grid = false;
      bool last_grid_is_ground = false;
      // 1 0 0 -1这样的模型认为是坑，1 1 -1 -1这样的认为是下水道
      for (auto it : v_ground_point_num) {
        if (it <= -1) {
          negative_grid_count++;
          no_negative_grid = false;
          // LOG(ERROR) << "negative_grid";
          last_grid_is_ground = false;

          // 无点云的栅格数大于一定值时认为认区域不可通行
          if (free_grid_count > i_drainage_count_
              //  || negative_grid_count > i_down_count_
          ) {
            has_unpassable_grid = true;
          }
          ground_grid_count = 0;
          free_grid_count = 0;
        } else if (it == 1) {
          ground_grid_count++;
          free_grid_count = 0;
          last_grid_is_ground = true;
        } else if (it == 0) {
          if (last_grid_is_ground && ground_grid_count > i_ground_count_) {
            free_grid_count++;
          } else {
            ground_grid_count = 0;
          }
        }
        // if (down_count > 6) {
        //   down = true;
        // }
      }

      if (has_unpassable_grid) {
        for (size_t i = 0; i < seg_cloud_part_y->size(); i++) {
          if (seg_cloud_part_y->points[i].z < -0.06 ||
              // z's value equal 0 is the ground points
              fabs(seg_cloud_part_y->points[i].z) > 0.0001) {
            // set a positive value so it can be detected in costmap
            seg_cloud_part_y->points[i].z = 0.7;
            ptr_curb->points.push_back(seg_cloud_part_y->points[i]);
          }
        }

        if (b_print_log_) {
          std::cout << "=========be care not to fall down ========";
          std::cout << -2.0 + seg_step_y * seg_y;
          for (auto it : v_ground_point_num) {
            if (it <= -1) {
              std::cout << it << " ";
            } else if (it == 1) {
              std::cout << 1 << " ";
            } else {
              std::cout << 0 << " ";
            }
          }
          std::cout << std::endl;
          LOG(ERROR) << "detected pit";
        }
      }
      // if (negative_grid_count > 0) {

      // }
      // else {
      //   *ptr_ground_sum_ += *seg_cloud_part_y;
      // }
    }

    // 如果不存在负向点，把0.06m以上的点作为障碍点，
    // 因为有下水道时有可能会有一些较高的突起，所以之前阙值设得较大
    if (no_negative_grid) {
      for (size_t i = 0; i < ptr_curb_sum_->size(); i++) {
        if (ptr_curb_sum_->points[i].z > d_curb_threshold_) {
          ptr_curb->points.push_back(ptr_curb_sum_->points[i]);
        }
      }
    }

    if (curb_sum_) {
      *ptr_not_ground_sum_ += *ptr_curb;
    }

    // LOG(ERROR) << ptr_not_ground_sum_->size();
    if (b_downsample_ && ptr_not_ground_sum_->size() != 0) {
      voxel_filter_.setInputCloud(ptr_not_ground_sum_);
      voxel_filter_.setLeafSize(0.06, 0.06, 0.04);
      voxel_filter_.filter(*ptr_not_ground_sum_);
    }

    // LOG(ERROR) << "size: " << ptr_not_ground_sum_->size();

    if (b_remove_outlier_ && ptr_not_ground_sum_->size() != 0) {
      outlier_removal_.setInputCloud(ptr_not_ground_sum_);
      outlier_removal_.setMeanK(i_min_neighbors_in_radius_);
      outlier_removal_.setStddevMulThresh(d_filter_search_radius_);
      outlier_removal_.filter(*ptr_not_ground_sum_);
    }

    static pcl::PointCloud<VPoint>::Ptr ptr_clustered_cloud(
        new pcl::PointCloud<VPoint>);
    ptr_clustered_cloud->clear();
    ptr_clustered_cloud->reserve(ptr_not_ground_sum_->size());

    // remove some single points and points which can be detected by 2d laser.
    if (ptr_not_ground_sum_->size() != 0) {
      clusterExtraction(ptr_not_ground_sum_, ptr_clustered_cloud);
    }

    // if (b_remove_outlier_ && ptr_clustered_cloud->size() != 0) {
    //   outlier_removal_.setInputCloud(ptr_clustered_cloud);
    //   outlier_removal_.setRadiusSearch(d_filter_search_radius_);
    //   outlier_removal_.setMinNeighborsInRadius(i_min_neighbors_in_radius_);
    //   outlier_removal_.filter(*ptr_clustered_cloud);
    // }
    if (b_ground_sum_) {
      // *ptr_not_ground_sum_ += *ptr_ground_sum_;
      *ptr_clustered_cloud += *ptr_ground_sum_;
    }

    // LOG(ERROR) << not_ground->size();

    // publishCloudMsg(ptr_not_ground_sum_, ptr_ground_sum_, ptr_curb_sum_);
    publishCloudMsg(ptr_clustered_cloud, ptr_ground_sum_, ptr_curb_sum_);
    ptr_not_ground_sum_->clear();
    ptr_ground_sum_->clear();
    ptr_curb_sum_->clear();

    // ptr_not_ground_sum_->reserve(ptr_cloud_processed->size() * cloud_sum_);
    // ptr_curb_sum_->reserve(ptr_cloud_processed->size() * cloud_sum_);
    // ptr_ground_sum_->reserve(ptr_cloud_processed->size() * cloud_sum_);

    cloud_count_ = 0;
  }

  // end = clock();
  // float runTime = ((float)(end - start)) / CLOCKS_PER_SEC;
  // LOG(INFO) << "Time of Ground Detection: " << runTime;
}

void GroundPlaneFit::clusterExtraction(const VPointsPtr &ptr_not_ground_cloud,
                                       const VPointsPtr &ptr_clusterd_cloud) {
  std::lock_guard<std::mutex> lock(mutex_);

  static std::vector<pcl::PointCloud<VPoint>> cluster_points;
  cluster_points.clear();
  cluster_points.reserve(ptr_not_ground_cloud->size());

  uniform_sampling_filter_.setInputCloud(ptr_point_cloud_laser_);
  uniform_sampling_filter_.setRadiusSearch(0.03);
  uniform_sampling_filter_.filter(*ptr_point_cloud_laser_);

  ptr_point_cluster_->setInputCloud(ptr_not_ground_sum_,
                                    ptr_point_cloud_laser_);

  static std::vector<int> v_index;
  v_index.clear();
  v_index.reserve(150);

  ptr_point_cluster_->clusteringFunction(cluster_points, v_index);

  publishBoxMsg(cluster_points, v_index);
  static pcl::KdTreeFLANN<VPoint> kdtree_laser;

  if (ptr_point_cloud_laser_->size() != 0) {
    kdtree_laser.setInputCloud(ptr_point_cloud_laser_);
  }

  int j = 0;
  for (std::vector<pcl::PointCloud<VPoint>>::iterator it =
           cluster_points.begin();
       it != cluster_points.end(); ++it) {
    int count = 0;

    static pcl::PointCloud<VPoint>::Ptr tmp_cloud(new pcl::PointCloud<VPoint>);
    tmp_cloud->clear();
    tmp_cloud->reserve(it->size());

    bool b = true;

    for (size_t i = 0; i < it->size(); i++) {
      VPoint vp;
      vp = it->points[i];
      vp.z = 0.0;
      tmp_cloud->push_back(vp);
    }

    std::vector<int> vi_point_index_remove;
    vi_point_index_remove.reserve(tmp_cloud->size());

    for (unsigned int pit = 0; pit < tmp_cloud->size(); pit++) {
      std::vector<int> vi_point_index_in_radius;
      std::vector<float> vf_squared_distance;

      if (ptr_point_cloud_laser_->size() != 0) {
        kdtree_laser.radiusSearch(tmp_cloud->points[pit], 0.25,
                                  vi_point_index_in_radius,
                                  vf_squared_distance);
      }
      if (vi_point_index_in_radius.size() > 0) {
        vi_point_index_remove.push_back(pit);
        count++;
      }

      if (b_remove_laser_plane_) {
        if (count > 10) {
          b = false;
          if (!b_remove_laser_plane_in_radius_) {
            break;
          }
        }
      }
    }
    if (b) {
      // += and push_back() only can use one , must be unified.
      for (unsigned int pit = 0; pit < it->size(); pit++) {
        ptr_clusterd_cloud->points.push_back(it->points[pit]);
      }
      // *ptr_clusterd_cloud += *it;
    } else {
      if (b_remove_laser_plane_in_radius_) {
        if (it->size() - vi_point_index_remove.size() < 10) {
          continue;
        }

        for (unsigned int pit = 0; pit < it->size(); pit++) {
          if (!isElementInVector(vi_point_index_remove, pit)) {
            ptr_clusterd_cloud->points.push_back(it->points[pit]);
          }
        }
      }
    }
    j++;
  }
  ptr_point_cloud_laser_->clear();
}

void GroundPlaneFit::publishBoxMsg(
    const std::vector<pcl::PointCloud<VPoint>> &Fitting_Points,
    std::vector<int> &v_index) {
  // 用以在rviz中可视化bounding box, 该类型中保存了所有bounding
  // box的中心点和三维尺寸
  visualization_msgs::msg::MarkerArray boxset_msgs;
  geometry_msgs::msg::PoseArray poses_msg;
  ptr_point_cluster_->FindingBox(Fitting_Points, v_index, boxset_msgs,
                                 poses_msg);
  // boxset_msgs.header.frame_id = "camera";
  boundmsgs_pub_->publish(boxset_msgs);
  // trajectory_pub_->publish(poses_msg);
}
void GroundPlaneFit::publishCloudMsg(const VPointsPtr &ptr_not_ground_cloud,
                                     const VPointsPtr &ptr_ground_cloud,
                                     const VPointsPtr &ptr_curb_cloud) {
  sensor_msgs::msg::PointCloud2 ground_msg;
  pcl::toROSMsg(*ptr_ground_cloud, ground_msg);
  ground_msg.header.stamp = ptr_clock_->now();
  ground_msg.header.frame_id = "camera";
  ground_points_pub_->publish(ground_msg);

  sensor_msgs::msg::PointCloud2 groundless_msg;
  sensor_msgs::msg::PointCloud cloud_scan_msg;
  pcl::toROSMsg(*ptr_not_ground_cloud, groundless_msg);
  groundless_msg.header.stamp = ptr_clock_->now();
  groundless_msg.header.frame_id = "camera";
  convertPointCloud2ToPointCloud(groundless_msg, cloud_scan_msg);
  cloud_scan_pub_->publish(cloud_scan_msg);

  sensor_msgs::msg::PointCloud2 curb_msg;
  pcl::toROSMsg(*ptr_curb_cloud, curb_msg);
  curb_msg.header.stamp = ptr_clock_->now();
  curb_msg.header.frame_id = "camera";
  curb_point_pub_->publish(curb_msg);
}

void GroundPlaneFit::curbSegment(const VPointsPtr &ptr_ground,
                                 const VPointsPtr &ptr_positive_curb,
                                 const VPointsPtr &ptr_ground_curb,
                                 std::vector<int> &curb_index) {
  pcl::KdTreeFLANN<VPoint> kdtree;
  kdtree.setInputCloud(ptr_ground);
  static VPoint searchPoint;

  std::vector<int> up_index;
  std::vector<int> down_index;
  std::vector<int> middle_index;
  middle_index.reserve(100);
  for (size_t i = 0; i < ptr_ground->points.size(); ++i) {
    searchPoint = ptr_ground->points[i];
    if (fabs(searchPoint.x) > 10
        // || fabs(searchPoint.z) < 0.02
    ) {
      continue;
    }

    static int K = i_search_k_;
    static std::vector<int> pointIdxNKNSearch;
    pointIdxNKNSearch.clear();
    pointIdxNKNSearch.reserve(K);
    static std::vector<float> pointNKNSquaredDistance;
    pointNKNSquaredDistance.clear();
    pointNKNSquaredDistance.reserve(K);

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0) {
      for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j) {
        static float dis;
        if (b_curb_negative_) {
          dis =
              fabs(searchPoint.z - ptr_ground->points[pointIdxNKNSearch[j]].z);
        } else {
          dis = searchPoint.z - ptr_ground->points[pointIdxNKNSearch[j]].z;
        }
        if (sqrt(pointNKNSquaredDistance[j]) > 0.5) {
          continue;
        }
        if (dis > curb_z_) {
          // curb_pc.points.push_back(searchPoint);
          // if (searchPoint.z > ptr_ground->points[pointIdxNKNSearch[j]].z) {
          bool in_flag = false;
          if (curb_index.size() > 0) {
            in_flag = isElementInVector(curb_index, i);
          }
          if (!in_flag) {
            if (searchPoint.z < d_positive_threshold_) {
              ptr_ground_curb->points.push_back(searchPoint);
            } else {
              ptr_positive_curb->points.push_back(searchPoint);
            }

            //     // curb_point_cloud.points.push_back(searchPoint);
            curb_index.push_back(i);
            //   }
            // } else {
            //   bool in_flag = false;
            //   if (curb_index.size() > 0) {
            //     in_flag = isElementInVector(curb_index, i);
            //   }
            //   if (!in_flag) {
            //     ptr_ground_curb->points.push_back(searchPoint);
            //     // curb_point_cloud.points.push_back(
            //     //     ptr_ground->points[pointIdxNKNSearch[j]]);
            //     curb_index.push_back(i);
            //   }
            // }
            break;
          }
        }
      }
    }
  }
}

void GroundPlaneFit::groundSegmentFurther(const VPointsPtr &ptr_ground,
                                          const VPointsPtr &ptr_positive_curb,
                                          const VPointsPtr &ptr_negative_curb,
                                          const VPointsPtr &ptr_ground_sum) {
  static std::vector<int> curb_index;
  curb_index.clear();
  curb_index.reserve(ptr_ground->size());

  static VPointsPtr ptr_ground_points(new pcl::PointCloud<VPoint>);
  ptr_ground_points->clear();
  ptr_ground_points->reserve(300);

  static VPointsPtr ptr_positive(new pcl::PointCloud<VPoint>);
  ptr_positive->clear();
  ptr_positive->reserve(300);

  static VPointsPtr ptr_not_poisitive_cloud(new pcl::PointCloud<VPoint>);
  ptr_not_poisitive_cloud->clear();
  ptr_not_poisitive_cloud->reserve(500);

  if (ptr_ground->size() != 0) {
    curbSegment(ptr_ground, ptr_positive, ptr_not_poisitive_cloud, curb_index);
  }

  for (size_t i = 0; i < ptr_ground->size(); i++) {
    if (!isElementInVector(curb_index, i)) {
      static VPoint point;
      point = ptr_ground->points[i];

      if (b_set_ground_zero_) {
        point.z = 0.0;
      }

      // put ground points insides 4.0m into ptr_negative_curb to detect
      // drainage.
      if (point.x < 4.0) {
        ptr_not_poisitive_cloud->push_back(point);
      }

      ptr_ground_points->push_back(point);
    }
  }

  // estimate the sensors z and pitch.
  // estimatePlane(ptr_ground_points);

  *ptr_positive_curb += *ptr_positive;
  *ptr_negative_curb += *ptr_not_poisitive_cloud;

  *ptr_ground_sum += *ptr_ground_points;
}

/*
    The function to filter by grid height
    1. grid init
    2. fill the grid
    3. pre segment of pointcloud
*/
void GroundPlaneFit::gridMapFilter(
    const pcl::PointCloud<VPoint>::ConstPtr ptr_cloud_in,
    const VPointsPtr ptr_no_ground, const VPointsPtr ptr_ground) {
  // grid map init
  static int grid_length = 2 * region_intesest_ / grid_resolution_;

  // fill the grid计算点位于哪个网格，填充
  for (int j = 0; j < grid_length; ++j) {
    for (int i = 0; i < grid_length; ++i) {
      grid_pts_[j][i].clear();
      grid_pts_[j][i].reserve(400);
    }
  }

  static VPoint tmp_pt;
  for (size_t i = 0; i < ptr_cloud_in->size(); ++i) {
    tmp_pt = ptr_cloud_in->points[i];

    static int GridI, GridJ;
    GridI = (region_intesest_ - tmp_pt.x) / grid_resolution_;
    GridJ = (region_intesest_ - tmp_pt.y) / grid_resolution_;
    grid_pts_[GridI][GridJ].push_back(tmp_pt);
  }

  // calculate the grid height
  for (int j = 0; j < grid_length; ++j) {
    for (int i = 0; i < grid_length; ++i) {
      if (grid_pts_[j][i].empty()) {
        continue;
      }

      // 有更高效的方法?
      sort(grid_pts_[j][i].begin(), grid_pts_[j][i].end(),
           pointCmp);  //按高度从小到大排序
      std::vector<VPoint>::iterator max = grid_pts_[j][i].end() - 1;
      std::vector<VPoint>::iterator min = grid_pts_[j][i].begin();
      float Height_dif = max->z - min->z;  // 网格中的高度差
      if (Height_dif < th_grid_het_ && max->z < sensor_height_) {
        //根据高度差阈值和高度阈值，初步将点分为地面点和非地面点
        for (size_t k = 0; k < grid_pts_[j][i].size(); ++k) {
          ptr_ground->push_back(grid_pts_[j][i][k]);
        }
      } else {
        for (size_t k = 0; k < grid_pts_[j][i].size(); ++k) {
          if (grid_pts_[j][i][k].z > 0.1) {
            ptr_no_ground->push_back(grid_pts_[j][i][k]);
          } else {
            ptr_ground->push_back(grid_pts_[j][i][k]);
          }
        }
      }
    }
  }
}

void GroundPlaneFit::polarFilter(pcl::PointCloud<VPoint> &p_ground,
                                 pcl::PointCloud<VPoint> &p_no_ground) {
  // ploar grid init
  double polar_rad =
      (polar_angle_ / 180.) * M_PI;  // polar_angle_ default value 0.2
  const int b_sizes = 2 * M_PI / polar_rad;
  // calculate the lowst distance of unground points
  std::vector<std::vector<VPoint>> clusters;
  clusters.resize(b_sizes);
  std::vector<float> min_dist;
  min_dist.resize(b_sizes, 0.0);
  for (size_t i = 0; i < p_no_ground.points.size(); i++) {
    VPoint tmp_pt = p_no_ground.points[i];
    double tmp_a = computBeta(tmp_pt);
    int index = tmp_a / polar_rad;
    if (index == b_sizes)
      index--;
    if (tmp_pt.z < sensor_height_)
      clusters[index].push_back(tmp_pt);
  }
  for (int j = 0; j < b_sizes; j++) {
    sort(clusters[j].begin(), clusters[j].end(), distCmp);
    if (clusters[j].size() != 0) {
      std::vector<VPoint>::iterator iter = clusters[j].begin();
      min_dist[j] = sqrt(iter->x * iter->x + iter->y * iter->y);
    } else {
      min_dist[j] = 999;
    }
  }
  // judge the points class
  pcl::PointCloud<VPoint> tmp_points;
  for (size_t i = 0; i < p_ground.points.size(); i++) {
    VPoint t_point = p_ground.points[i];
    double tmp_b = computBeta(t_point);
    float dis_p = sqrt(t_point.x * t_point.x + t_point.y * t_point.y);
    int ind = tmp_b / polar_rad;
    if (ind == b_sizes)
      ind--;
    if (dis_p < min_dist[ind]) {
      tmp_points.push_back(t_point);
    }
  }
  p_ground.clear();
  p_ground = tmp_points;
}

/*
    Extract initial seeds of the given pointcloud sorted segment.
    p_sorted: sorted pointcloud
    num_lpr_: num of LPR
    th_seeds_: threshold distance of seeds
*/
void GroundPlaneFit::extractInitialSeeds(const VPointsPtr &ptr_ground,
                                         const VPointsPtr &ptr_g_cloud_seeds) {
  // LPR is the mean of low point representative
  static double sum;
  sum = 0;
  static int cnt;
  cnt = 0;
  static unsigned int i;
  static double lpr_height;

  sort(ptr_ground->points.begin(), ptr_ground->points.end(),
       pointCmp);  // 将地面点按照高度从小到大排序
  // Calculate the mean height value.

  if (ptr_ground->points.size() != 0) {
    if (fabs(ptr_ground->points.front().z) - fabs(ptr_ground->points.back().z) <
        0.0) {
      for (i = 0; i < ptr_ground->points.size() && cnt < num_lpr_; i++) {
        // num of lpr default 20,前20个较低的点
        sum += ptr_ground->points[i].z;
        cnt++;
      }
    } else {
      for (i = ptr_ground->points.size() - 1; i > 0 && cnt < num_lpr_; i--) {
        // num of lpr default 20,前20个较低的点
        sum += ptr_ground->points[i].z;
        cnt++;
      }
    }
  }

  lpr_height = cnt != 0 ? sum / cnt : 0;
  // lpr_height = 0.0;  // cnt != 0 ? sum / cnt : 0;

  // if (ptr_ground->points.size() != 0) {
  //   lpr_height = ptr_ground->points[ptr_ground->points.size() / 2].z;

  // } else {
  //   lpr_height = 0;
  // }

  ptr_g_cloud_seeds->clear();
  ptr_g_cloud_seeds->reserve(ptr_ground->points.size());

  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (i = 0; i < ptr_ground->points.size(); i++) {
    if (fabs(ptr_ground->points[i].z - lpr_height) <
        th_seeds_) {  // th_seeds_ default 0.4
      ptr_g_cloud_seeds->points.push_back(ptr_ground->points[i]);
    }
  }
}

/*
    The function to estimate plane model.
    The main step is performed SVD on covariance matrix.
*/
void GroundPlaneFit::estimatePlane(const VPointsPtr &ptr_ground_cloud) {
  // Create covarian matrix.
  // 1. calculate (x,y,z) mean
  static float x_mean, y_mean, z_mean;
  static size_t i;

  x_mean = 0;
  y_mean = 0;
  z_mean = 0;

  for (i = 0; i < ptr_ground_cloud->points.size(); i++) {
    x_mean += ptr_ground_cloud->points[i].x;
    y_mean += ptr_ground_cloud->points[i].y;
    z_mean += ptr_ground_cloud->points[i].z;
  }
  // incase of divide zero
  int size = ptr_ground_cloud->points.size() != 0
                 ? ptr_ground_cloud->points.size()
                 : 1;
  x_mean /= size;
  y_mean /= size;
  z_mean /= size;

  // 2. calculate covariance
  // cov(x,x), cov(y,y), cov(z,z)
  // cov(x,y), cov(x,z), cov(y,z)
  static float xx, yy, zz;
  static float xy, xz, yz;

  xx = 0;
  yy = 0;
  zz = 0;
  xy = 0;
  xz = 0;
  yz = 0;

  for (i = 0; i < ptr_ground_cloud->points.size(); i++) {
    xx += (ptr_ground_cloud->points[i].x - x_mean) *
          (ptr_ground_cloud->points[i].x - x_mean);
    xy += (ptr_ground_cloud->points[i].x - x_mean) *
          (ptr_ground_cloud->points[i].y - y_mean);
    xz += (ptr_ground_cloud->points[i].x - x_mean) *
          (ptr_ground_cloud->points[i].z - z_mean);
    yy += (ptr_ground_cloud->points[i].y - y_mean) *
          (ptr_ground_cloud->points[i].y - y_mean);
    yz += (ptr_ground_cloud->points[i].y - y_mean) *
          (ptr_ground_cloud->points[i].z - z_mean);
    zz += (ptr_ground_cloud->points[i].z - z_mean) *
          (ptr_ground_cloud->points[i].z - z_mean);
  }

  // 3. setup covarian matrix cov
  MatrixXf cov(3, 3);
  cov << xx, xy, xz, xy, yy, yz, xz, yz, zz;
  cov /= size;

  // Singular Value Decomposition: SVD
  JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));

  // mean ground seeds value
  MatrixXf seeds_mean(3, 1);
  seeds_mean << x_mean, y_mean, z_mean;

  d_mean_z_ = z_mean;
  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
  LOG(ERROR) << z_mean << "normal: " << normal_(0) << " , " << normal_(1)
             << " , " << normal_(2);

  // set distance threhold to `th_dist - d`
  // if (d_ < 0.0) {
  th_dist_d_ = th_dist_ - d_;
  // } else {
  //   th_dist_d_ = 0.5 - d_;
  // }
}

void GroundPlaneFit::startOrbbecTimer() {
  // get_clouds_timer_ = node_->create_wall_timer(
  //     std::chrono::milliseconds(static_cast<int>(10)),
  //     std::bind(&GroundPlaneFit::getAndPubCloudsCB, this));
}

void GroundPlaneFit::getAndPubCloudsCB() {
  pcl::PointCloud<VPoint> cloud_in;
  bool has_message = false;
  m_pMtx->lock();
  cloud_in.points.resize(m_pData->point_size);
  if (m_pData->i_work == 1) {
    has_message = true;
    for (size_t i = 0; i < m_pData->point_size; i++) {
      cloud_in.points[i].x = m_pData->points[i].p_x;
      cloud_in.points[i].y = m_pData->points[i].p_y;
      cloud_in.points[i].z = m_pData->points[i].p_z;
      cloud_in.points[i].intensity = m_pData->points[i].p_i;
    }
  }
  m_pData->i_work = 0;
  m_pMtx->unlock();
  if (has_message) {
    // std::cout << "========clouds size: " << cloud_in.points.size() <<
    // std::endl;
    rclcpp::Clock::SharedPtr ptr_clock_;
    ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    sensor_msgs::msg::PointCloud2 groundless_msg;
    sensor_msgs::msg::PointCloud cloud_scan_msg;
    pcl::toROSMsg(cloud_in, groundless_msg);
    groundless_msg.header.stamp = ptr_clock_->now();
    groundless_msg.header.frame_id = "camera";
    convertPointCloud2ToPointCloud(groundless_msg, cloud_scan_msg);
    orbbec_pub_->publish(cloud_scan_msg);
  }
}

void GroundPlaneFit::livoxCallback_recon(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  clock_t start, end;
  start = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempPointsCloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> All_PointFrame;

  // 将接受到的sensor_msgs::PointCloud2的消息转换为pcl::PointCloud<T>的点云格式
  pcl::fromROSMsg(*cloud_msg, *tempPointsCloud);
  // 预处理过程，选择环境空间中x轴(0, 20)和y轴(-12, 12)的范围
  Point_Cloud_Filter Point_Filter;
  Point_Filter.setInputCloud(tempPointsCloud);
  Point_Filter.passPointFilter(0.0, 10, "x", *tempPointsCloud);
  Point_Filter.passPointFilter(-6, 6, "y", *tempPointsCloud);
  Point_Filter.voxelGridFilter(0.04, 0.04, 0.04, *tempPointsCloud);

  // 为待处理的点云创建kd树的搜索方式
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(tempPointsCloud);

  // 高度差和平滑度滤波器进行滤波
  Point_Filter.heightDifferenceFilter(kdtree, 0.08, 0.05, 0.15, 0.008);
  Point_Filter.smoothNeighborFilter(kdtree, 0.08, 0.008);

  // 将处理后的点云保存
  All_PointFrame.push_back(*tempPointsCloud);

  cloud_count_++;
  rclcpp::Clock::SharedPtr ptr_clock_;
  ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  if (cloud_count_ == cloud_sum_) {
    for (unsigned int i = 0; i < All_PointFrame.size(); i++) {
      filter_sum_ += All_PointFrame[i];
    }
    sensor_msgs::msg::PointCloud2 groundless_msg;
    sensor_msgs::msg::PointCloud cloud_scan_msg;
    pcl::toROSMsg(filter_sum_, groundless_msg);
    groundless_msg.header.stamp = ptr_clock_->now();
    groundless_msg.header.frame_id = "livox_frame";
    convertPointCloud2ToPointCloud(groundless_msg, cloud_scan_msg);
    cloud_scan_pub_->publish(cloud_scan_msg);

    filter_sum_.clear();
    All_PointFrame.clear();
    cloud_count_ = 0;
  }
  end = clock();
  float runTime = ((float) (end - start)) / CLOCKS_PER_SEC;
  LOG(INFO) << "Time of Ground Detection: " << runTime;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  GroundPlaneFit ground;
  // ground.startOrbbecTimer();
  rclcpp::spin(ground.getNodeHander());

  // boost::interprocess::shared_memory_object::remove("shm");
  rclcpp::shutdown();
  return 0;
}