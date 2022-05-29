#ifndef GROUND_HPP_
#define GROUND_HPP_
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ground_utils.hpp"

#include <memory>
#include <mutex>

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

const int MAX_POINT_NUM = 300000;

struct CloudPoint {
  double p_x;
  double p_y;
  double p_z;
  double p_i;
};

typedef struct shared_use_st {
  CloudPoint points[MAX_POINT_NUM];
  size_t point_size;
  int i_work;
} shared_use_st;

namespace CVTE_BABOT {
class PointCloudCluster;
}

class GroundPlaneFit {
 public:
  GroundPlaneFit();

  void updateParameter();

  void startOrbbecTimer();

  inline rclcpp::Node::SharedPtr getNodeHander() { return node_; }

 private:
  void livoxCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud_msg);

  void livoxCallback_recon(
      const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  void transformAndFilterCloud(const VPointsPtr &ptr_cloud_raw,
                               const VPointsPtr &ptr_cloud_processed);

  void gridMapFilter(const pcl::PointCloud<VPoint>::ConstPtr in_cloud_ptr,
                     const VPointsPtr ptr_no_ground,
                     const VPointsPtr ptr_ground);

  void extractInitialSeeds(const VPointsPtr &ptr_ground,
                           const VPointsPtr &ptr_g_cloud_seeds);

  void estimatePlane(const VPointsPtr &ptr_groud);

  void curbSegment(const VPointsPtr &ptr_ground,
                   const VPointsPtr &ptr_positive_curb,
                   const VPointsPtr &ptr_groud_curb,
                   std::vector<int> &curb_index);

  void clusterExtraction(const VPointsPtr &ptr_not_ground_cloud,
                         const VPointsPtr &ptr_clusterd_cloud);

  void publishCloudMsg(const VPointsPtr &ptr_not_ground_cloud,
                       const VPointsPtr &ptr_ground_cloud,
                       const VPointsPtr &ptr_curb_cloud);

  void groundSegmentFurther(const VPointsPtr &ptr_ground,
                            const VPointsPtr &ptr_positive_curb,
                            const VPointsPtr &ptr_negative_curb,
                            const VPointsPtr &ptr_ground_sum);

  void polarFilter(pcl::PointCloud<VPoint> &p_ground,
                   pcl::PointCloud<VPoint> &p_no_ground);

  void publishBoxMsg(const std::vector<pcl::PointCloud<VPoint>> &Fitting_Points,
                     std::vector<int> &v_index);

  void laserscanReceived(const sensor_msgs::msg::LaserScan::SharedPtr);

  /**
   * getAndPubCloudsCB
   * @brief
   * 获取点云并发布
   *
   * @param[in] none
   * @param[out] none
   * */
  void getAndPubCloudsCB();

  rclcpp::Node::SharedPtr node_;  ///< 用于收发数据的节点
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cloud_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr orbbec_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      ground_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curb_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      boundmsgs_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      livox_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_sub_;                             ///< 激光雷达数据
  rclcpp::TimerBase::SharedPtr get_clouds_timer_;  ///< 定时器
  rclcpp::Clock::SharedPtr ptr_clock_;
  pcl::PointCloud<pcl::PointNormal>::Ptr ptr_cloud_with_normals_ =
      pcl::PointCloud<pcl::PointNormal>::Ptr(
          new pcl::PointCloud<pcl::PointNormal>);

  std::string s_topic_name_;  ///< 3d激光topic名
  bool b_fusion_2d_laser_;    ///< 是否融合2d激光数据提升聚类效果

  double sensor_height_;  ///< 传感器高度
  double d_sensor_pitch_;
  double d_sensor_x_;         ///< 相对于2d激光的x
  double d_region_filter_x_;  ///< 传感器高度
  int region_intesest_;       ///< 感兴趣区域
  double grid_resolution_;    ///< 栅格分辨率
  double th_grid_het_;        ///< 高度差阈值
  double d_mean_z_;

  int num_seg_x_;
  int num_seg_y_;
  int num_iter_;
  int num_lpr_;
  double th_seeds_;
  double th_dist_;
  double polar_angle_;
  float d_;
  MatrixXf normal_;
  float th_dist_d_;
  int cloud_count_;
  double search_radius_;
  double curb_z_;
  int cloud_sum_;
  bool curb_sum_;
  int i_search_k_;
  bool b_ground_sum_;
  bool b_set_ground_zero_;
  int i_min_neighbors_in_radius_;
  double d_filter_search_radius_;
  double d_cluster_tolerance_;

  double d_distance_close_;
  double d_distance_far_;

  double d_up_height_close_;
  double d_down_height_close_;

  double d_up_height_far_;
  double d_down_height_far_;

  int i_max_cluster_size_;
  bool b_remove_outlier_;
  bool b_downsample_;
  bool b_remove_laser_plane_;
  bool b_remove_laser_plane_in_radius_;

  bool b_pre_segment_;
  bool b_segment_all_cloud_;

  double d_remove_laser_plane_radius_;

  double d_increment_distance_radius_;
  double d_clusters_radius_;
  bool b_use_multi_radius_;
  bool b_use_last_cluters_;
  int i_min_cluster_size_;
  bool b_curb_negative_;

  int i_down_count_ = 0;
  int i_ground_count_ = 1;
  int i_drainage_count_ = 0;
  double d_positive_threshold_;
  double d_curb_threshold_;
  double d_further_threshold_;
  int i_further_count_;

  bool b_print_log_;

  std::shared_ptr<CVTE_BABOT::PointCloudCluster> ptr_point_cluster_;
  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud_laser_ = nullptr;
  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud_laser_copy_ = nullptr;

  std::vector<std::vector<std::vector<VPoint>>> grid_pts_;

  VPointsPtr ptr_no_ground_;
  VPointsPtr ptr_ground_;

  VPointsPtr ptr_ground_part_;

  VPointsPtr ptr_not_ground_sum_;
  VPointsPtr ptr_ground_sum_;
  VPointsPtr ptr_curb_sum_;

  pcl::PointCloud<pcl::PointXYZ> filter_sum_;
  pcl::VoxelGrid<VPoint> voxel_filter_;
  pcl::UniformSampling<VPoint> uniform_sampling_filter_;
  pcl::StatisticalOutlierRemoval<VPoint> outlier_removal_;

  boost::interprocess::managed_shared_memory m_managed_shm;
  boost::interprocess::interprocess_mutex *m_pMtx;
  shared_use_st *m_pData;

  std::mutex mutex_;
};

#endif  // GROUND_HPP_
