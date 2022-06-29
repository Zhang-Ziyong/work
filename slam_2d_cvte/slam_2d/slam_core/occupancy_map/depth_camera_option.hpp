#ifndef DEPTH_CAMERA_OPTIONS
#define DEPTH_CAMERA_OPTIONS

#include <string>
#include <vector>
#include <cmath>

namespace cvte_lidar_slam {
class DepthCameraOptions {
 public:
  bool use_depth_camera = true;
  bool do_calibration = true;
  std::string cloud_topic_name = "orbbec_cloud";
  std::vector<double> front_up_transform = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0};

  std::vector<double> front_down_transform = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0};

  double depth_dist_value = 0.23;
  double depth_seg_ratio = 0.7;
  double occ_v_limit = 0.01;
  double occ_w_limit = 0.01;

  double min_x = 0.1;
  double max_x = 2.0;
  double min_y = -1.2;
  double max_y = 1.2;

  double high_filter_value = 0.03;
};
struct DepthOccupancyMapOptions {
  std::vector<double> search_area_point = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double add_bel_value = 0.0;
  double reduce_bel_value = 0.0;
  double max_bel_value = 0.0;

  float laser_min_angle = -M_PI;
  float laser_max_angle = M_PI;
  float laser_min_range = 0.2;
  float laser_max_range = 20.0;
  // float laser_angle_increment = 1.0f / 180 * M_PI;
  float laser_angle_increment = 0.4f / 180 * M_PI;
  bool use_view_cloud = true;  // 使用点云增强地图细节
};
}  // namespace cvte_lidar_slam

#endif
