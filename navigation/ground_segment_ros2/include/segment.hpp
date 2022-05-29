#ifndef SEGMENT_HPP_
#define SEGMENT_HPP_

// 需要在open_cv头文件之前,不然编译报错
#include <deque>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ground_utils.hpp"
#include "voxel_grid_filter.hpp"

namespace CVTE_BABOT {
class GroundSegment {
 public:
  GroundSegment();

  void updateParameter();

  inline rclcpp::Node::SharedPtr getNodeHander() { return node_; }

 private:
  void pointCloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud_msg);

  void transformAndFilterCloud(const VPointsPtr &ptr_cloud_raw,
                               const VPointsPtr &ptr_cloud_processed);

  void gridMapFilter(const pcl::PointCloud<VPoint>::ConstPtr in_cloud_ptr,
                     const VPointsPtr ptr_no_ground,
                     const VPointsPtr ptr_ground,
                     const VPointsPtr ptr_static_cloud);

  void computeAndPubLaserScan(const VPointsPtr &ptr_obstacle_cloud,
                              const builtin_interfaces::msg::Time &time_stamp);

  void printCoverCellsInMap(
      const std::vector<std::vector<uint32_t>> &vvui_point_cells);

  void setPointToMap(const double &x, const double &y,
                     std::vector<std::vector<uint32_t>> &vvui_point_cells);

  void resetMap(std::vector<std::vector<uint32_t>> &vvui_point_cells);

  void updateScanMsg(const double &x, const double &y,
                     sensor_msgs::msg::LaserScan &laser_scan_msg);

  rclcpp::Node::SharedPtr node_;  ///< 用于收发数据的节点
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curb_point_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_sub_;  ///< 激光雷达数据

  sensor_msgs::msg::LaserScan laser_scan_msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_ = nullptr;

  std::string s_topic_name_;  ///< 3d激光topic名

  double sensor_height_;              ///< 传感器高度
  double d_region_filter_threshold_;  ///< x或y低于这个值的点滤掉
  int region_interest_;               ///< 感兴趣区域
  double grid_resolution_;            ///< 栅格分辨率
  double th_grid_het_;                ///< 高度差阈值

  double curb_z_;
  int cloud_sum_;
  bool curb_sum_;
  int i_search_k_;
  bool b_set_ground_zero_;

  bool b_downsample_;

  bool b_print_log_;

  std::vector<std::vector<std::vector<VPoint>>> grid_pts_;
  std::vector<std::vector<VPoint> *> vvp_grid_filled_;

  VPointsPtr ptr_no_ground_;
  VPointsPtr ptr_ground_;

  VPointsPtr ptr_not_ground_sum_;
  VPointsPtr ptr_ground_sum_;

  VPointsPtr ptr_cloud_raw_;
  VPointsPtr ptr_cloud_processed_;
  std::shared_ptr<VoxelGridFilter> voxel_filter_no_ground_;
  std::shared_ptr<VoxelGridFilter> voxel_filter_ground_;

  std::vector<std::vector<uint32_t>> vv_point_cells_;

  bool b_remove_outlier_;
  int i_min_neighbors_in_radius_;
  double d_remove_outlier_radius_;
  pcl::RadiusOutlierRemoval<VPoint> outlier_removal_;
};
}  // namespace CVTE_BABOT

#endif  // SEGMENT_HPP_
