#ifndef DEPTH_CAMERA_OPTIONS
#define DEPTH_CAMERA_OPTIONS

#include <string>
#include <vector>

namespace slam2d_core {
namespace occupancy_map {
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
};
}  // namespace occupancy_map
}  // namespace slam2d_core

#endif