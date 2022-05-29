#include "recon_cluster.hpp"
#include <glog/logging.h>

#include <pcl/registration/icp.h>

namespace CVTE_BABOT {

PointCloudCluster::PointCloudCluster(const rclcpp::Clock::SharedPtr& ptr_clock)
    : ptr_clock_(ptr_clock) {
  vb_if_new_cluster_.resize(200, true);

  ptr_point_cloud_2d_ =
      pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
  ptr_point_cloud_laser_ =
      pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);

  ptr_point_cloud_and_laser_ =
      pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
}

void PointCloudCluster::setInputCloud(
    const pcl::PointCloud<VPoint>::Ptr& ptr_input_pointcloud,
    const pcl::PointCloud<VPoint>::Ptr& ptr_laser_pointcloud) {
  ptr_point_cloud_ = ptr_input_pointcloud;
  *ptr_point_cloud_2d_ = *ptr_point_cloud_;

  for (size_t i = 0; i < ptr_point_cloud_2d_->size(); i++) {
    ptr_point_cloud_2d_->points[i].z = 0.0;
  }

  *ptr_point_cloud_laser_ = *ptr_laser_pointcloud;

  *ptr_point_cloud_and_laser_ = *ptr_point_cloud_2d_ + *ptr_point_cloud_laser_;
  // LOG(ERROR) << ptr_point_cloud_and_laser_->size();
}

void PointCloudCluster::recursiveDiffusion(
    const pcl::KdTreeFLANN<VPoint>& kdtree, const VPoint& center_point,
    const int& label_name, const float& radius) {
  /**
   *  pointIdxRadiusSearch：点P的邻域之内的点的序列集
   *  pointRadiusSquaredDistance：邻域之内的点到点P的距离
   */
  std::vector<int> vi_point_index_in_radius;
  vi_point_index_in_radius.reserve(150);
  std::vector<float> vf_squared_distance;
  vf_squared_distance.reserve(150);

  double used_radius = radius;
  if (cluster_params_.use_multi_radius &&
      sqrt(center_point.x * center_point.x + center_point.y * center_point.y) >
          5.0) {
    used_radius += cluster_params_.increment_distance_radius;
  }

  kdtree.radiusSearch(center_point, used_radius, vi_point_index_in_radius,
                      vf_squared_distance);

  for (size_t i = 0; i < vi_point_index_in_radius.size(); i++) {
    if (vi_label_index_[vi_point_index_in_radius[i]] == -1) {
      vi_label_index_[vi_point_index_in_radius[i]] = label_name;

      // 超出说明是邻点是2d激光的点，只查找相邻的3d激光的点
      if (vi_point_index_in_radius[i] >= ptr_point_cloud_->size()) {
        recursiveDiffusion(
            kdtree_2d_,
            ptr_point_cloud_and_laser_->points[vi_point_index_in_radius[i]],
            label_name, radius);
      } else {
        // 若在邻域之内，若某一点无标签，将其对应的序列号在vi_label_index_中确定其标签
        vi_cluster_size_[label_name]++;

        // 查找相邻的3d激光与2d激光点
        recursiveDiffusion(
            kdtree_2d_with_laser_,
            ptr_point_cloud_2d_->points[vi_point_index_in_radius[i]],
            label_name, radius);
      }
    }
  }
}

void PointCloudCluster::updateParams(const ClusterParams& cp) {
  cluster_params_ = cp;
}

bool isElementInVector(std::vector<int> v, int element) {
  static std::vector<int>::iterator it;
  it = find(v.begin(), v.end(), element);
  if (it != v.end()) {
    return true;
  } else {
    return false;
  }
}

void PointCloudCluster::clusteringFunction(
    std::vector<pcl::PointCloud<VPoint>>& output_fitting_points,
    std::vector<int>& v_index_output) {
  output_fitting_points.clear();

  vi_label_index_.clear();
  vi_label_index_.resize(ptr_point_cloud_and_laser_->size(), -1);

  /**
   * 根据点云的大小初始化vi_label_index_为-1，且-1表示该点未被标记
   */

  vi_cluster_size_.clear();
  vi_cluster_size_.resize(ptr_point_cloud_and_laser_->size(), 0);

  /**
   * 为待处理的点云创建KD树的搜索方式
   */
  kdtree_2d_with_laser_.setInputCloud(ptr_point_cloud_and_laser_);
  kdtree_2d_.setInputCloud(ptr_point_cloud_2d_);
  assert(ptr_point_cloud_and_laser_->size() == vi_cluster_size_.size());

  // pcl::search::KdTree<VPoint>::Ptr search_tree(new
  // pcl::search::KdTree<VPoint>);

  // ptr_cloud_with_normals_->clear();
  // pcl::NormalEstimation<VPoint, pcl::PointNormal> ne;
  // ne.setInputCloud(ptr_point_cloud_);
  // ne.setSearchMethod(search_tree);
  // ne.setRadiusSearch(cluster_params_.clusters_radius);
  // ne.compute(*ptr_cloud_with_normals_);

  std::vector<int> v_index;
  /**
   * @brief Label_Name为标签的类别
   */
  // label_name_ = v_cluster_cloud_.size();
  if (v_index_.size() != 0 && cluster_params_.use_last_cluters) {
    for (size_t i = v_index_.size(); i > 0; i--) {
      static Eigen::Vector4f centroid;
      pcl::compute3DCentroid(v_cluster_cloud_[i - 1], centroid);
      static VPoint cluster_centroid;
      cluster_centroid.x = centroid[0];
      cluster_centroid.y = centroid[1];
      cluster_centroid.z = 0.0;

      static bool if_new_cluster = false;
      if_new_cluster = false;
      static int index;
      index = v_index_[i - 1];
      if (index < 30) {
        index += 30;
        if_new_cluster = true;
      }

      while (isElementInVector(v_index, index)) {
        index++;
      }

      vb_if_new_cluster_[index] = if_new_cluster;

      recursiveDiffusion(kdtree_2d_with_laser_, cluster_centroid, index,
                         cluster_params_.clusters_radius);

      v_index.push_back(index);
    }
  }

  label_name_ = 0;

  size_t i;
  for (i = 0; i < ptr_point_cloud_->size(); i++) {
    if (vi_label_index_[i] == -1) {
      while (isElementInVector(v_index, label_name_)) {
        label_name_++;
      }
      /**
       * 若vi_label_index_[i]为零，则该点需要进行标记
       */
      recursiveDiffusion(kdtree_2d_with_laser_, ptr_point_cloud_2d_->points[i],
                         label_name_, cluster_params_.radius);

      v_index.push_back(label_name_);
    }
  }

  std::sort(v_index.begin(), v_index.end());
  /**
   *  根据vi_label_index_的记录对点云进行分割，将不同类别的分割结果，
   *  作为单独的点云放于output_fitting_points点云集中。
   *  认为某一标签的聚类结果的数量小于某一阈值，则视聚类结果属于离散集合，无法构成一类。
   */

  static std::vector<pcl::PointCloud<VPoint>> v_all_cluster_points;
  v_all_cluster_points.clear();
  v_all_cluster_points.resize(v_index.back() + 1);

  for (i = 0; i < v_index.size(); i++) {
    v_all_cluster_points[v_index[i]].reserve(vi_cluster_size_[v_index[i]]);
  }

  for (i = 0; i < ptr_point_cloud_->size(); i++) {
    v_all_cluster_points.at(vi_label_index_[i])
        .push_back(ptr_point_cloud_->points[i]);
  }

  v_index_.clear();
  for (i = 0; i < v_index.size(); i++) {
    if (vi_cluster_size_[v_index[i]] > cluster_params_.min_cluster_size) {
      v_index_.push_back(v_index[i]);
    }
  }

  for (i = 0; i < v_index_.size(); i++) {
    output_fitting_points.push_back(v_all_cluster_points[v_index_[i]]);
  }

  v_index_output = v_index_;

  v_cluster_cloud_ = output_fitting_points;
  assert(v_cluster_cloud_.size() == v_index_.size());
}

void PointCloudCluster::FindingBox(
    const std::vector<pcl::PointCloud<VPoint>>& fitting_points,
    std::vector<int>& v_index,
    visualization_msgs::msg::MarkerArray& boxset_msgs,
    geometry_msgs::msg::PoseArray& poses_msg) {
  poses_msg.header.frame_id = "camera";  // will be reused by P-N experts

  int id = 0;
  for (size_t i = 0; i < fitting_points.size(); i++) {
    id = v_index[i];
    /**
     * temp_box：记录聚类结果中某一类的bounding box
     * GridBox：记录bounding box网格化分割成smaller box的结果
     *
     */
    visualization_msgs::msg::Marker temp_box;
    visualization_msgs::msg::Marker text_box;
    visualization_msgs::msg::Marker GridBox;
    visualization_msgs::msg::Marker trajectory_box;

    uint32_t shape;
    if (id >= 30) {
      shape = visualization_msgs::msg::Marker::CUBE;
      temp_box.color.r = 1.0f;
      temp_box.color.g = 0.0f;
      temp_box.color.b = 0.0f;
    } else {
      shape = visualization_msgs::msg::Marker::SPHERE;
      temp_box.color.r = 0.0f;
      temp_box.color.g = 0.0f;
      temp_box.color.b = 1.0f;
    }

    // temp_box.header.stamp = ros::Time::now();
    temp_box.header.frame_id = "camera";
    temp_box.ns = "box";
    temp_box.type = shape;
    temp_box.lifetime = rclcpp::Duration(1.0);
    temp_box.pose.orientation.x = 0.0;
    temp_box.pose.orientation.y = 0.0;
    temp_box.pose.orientation.z = 0.0;
    temp_box.pose.orientation.w = 1.0;
    temp_box.color.a = 0.5;

    // GridBox.header.stamp = ros::Time::now();
    GridBox.header.frame_id = "camera";
    GridBox.ns = "box";
    GridBox.type = shape;
    GridBox.pose.orientation.x = 0.0;
    GridBox.pose.orientation.y = 0.0;
    GridBox.pose.orientation.z = 0.0;
    GridBox.pose.orientation.w = 1.0;
    GridBox.color.r = 1.0f;
    GridBox.color.g = 1.0f;
    GridBox.color.b = 0.0f;
    GridBox.color.a = 0.5;

    trajectory_box.header.frame_id = "camera";
    trajectory_box.ns = "trajectory";
    trajectory_box.id = id;
    trajectory_box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_box.scale.x = 0.1;
    trajectory_box.color.a = 1.0;
    trajectory_box.color.r = std::max(0.3, static_cast<double>(id % 3) / 3.0);
    trajectory_box.color.g = std::max(0.3, static_cast<double>(id % 6) / 6.0);
    trajectory_box.color.b = std::max(0.3, static_cast<double>(id % 9) / 9.0);
    trajectory_box.lifetime = rclcpp::Duration(1.0);
    /**
     * centriodPoint记录点云集所形成bounding box的形心中心点坐标
     *
     */
    VPoint centriodPoint;

    /**
     * 记录bounding box的参数，三个方向的边界值
     *
     */
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    /**
     * 计算centriodPoint的中心坐标点坐标
     *
     */
    for (size_t j = 0; j < fitting_points[i].size(); j++) {
      centriodPoint.x += fitting_points[i][j].x;
      centriodPoint.y += fitting_points[i][j].y;
      centriodPoint.z += fitting_points[i][j].z;
      if (fitting_points[i][j].x < min_x) {
        min_x = fitting_points[i][j].x;
      }
      if (fitting_points[i][j].y < min_y) {
        min_y = fitting_points[i][j].y;
      }
      if (fitting_points[i][j].z < min_z) {
        min_z = fitting_points[i][j].z;
      }
      if (fitting_points[i][j].x > max_x) {
        max_x = fitting_points[i][j].x;
      }
      if (fitting_points[i][j].y > max_y) {
        max_y = fitting_points[i][j].y;
      }
      if (fitting_points[i][j].z > max_z) {
        max_z = fitting_points[i][j].z;
      }
    }
    centriodPoint.x /= fitting_points[i].size();
    centriodPoint.y /= fitting_points[i].size();
    centriodPoint.z /= fitting_points[i].size();

    /**
     * 计算bounding box消息类新所需要的参数值
     * length：长，若该类别点云集平行于Z轴方向分布，则定义其长为0.6m
     * width：宽，若该类别点云集平行于Z轴方向分布，则定义其宽为0.6m
     * heigth：高
     */
    float length = max_x - min_x;
    float width = max_y - min_y;
    float heigth = max_z - min_z;

    // if (max_z < 0.08 && length < 0.20 && width < 0.25) {
    //   LOG(ERROR) << "continue";
    //   continue;
    // }

    // if (length < 0.20 && width < 0.20) {
    //   continue;
    // }
    if (length == 0.0) {
      length = 0.6;
    }
    if (width == 0.0) {
      width = 0.6;
    }

    temp_box.pose.position.x = (max_x + min_x) / 2;
    temp_box.pose.position.y = (max_y + min_y) / 2;
    temp_box.pose.position.z = (max_z + min_z) / 2;

    temp_box.scale.x = ((length < 0) ? -1 * length : length);
    temp_box.scale.y = ((width < 0) ? -1 * width : width);
    temp_box.scale.z = ((heigth < 0) ? -1 * heigth : heigth);

    /**
     * 若该类别点云其整体的bounding box不大，则无需网格化为smaller
     bounding
     box
     *
     */
    // if (temp_box.scale.x < 1.6 && temp_box.scale.y < 1.6) {
    temp_box.id = id;
    boxset_msgs.markers.push_back(temp_box);

    geometry_msgs::msg::Point p;

    p.x = temp_box.pose.position.x;
    p.y = temp_box.pose.position.y;

    if (id >= 30 && !vb_if_new_cluster_[id]) {
      mv_previou_poses_[id].push_back(p);
      for (size_t k = 0; k < mv_previou_poses_[id].size(); k++) {
        trajectory_box.points.push_back(mv_previou_poses_[id][k]);
      }
      boxset_msgs.markers.push_back(trajectory_box);
    }

    if (id >= 30 && vb_if_new_cluster_[id]) {
      mv_previou_poses_[id].clear();
      // mv_previou_poses_[id].push_back(p);
    }

    if (id >= 30) {
      text_box.ns = "id";
      text_box.id = id;
      text_box.header.frame_id = "camera";
      text_box.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_box.pose.position.x = temp_box.pose.position.x;
      text_box.pose.position.y = temp_box.pose.position.y;
      text_box.pose.position.z = 1.2;
      text_box.scale.z = 0.5;
      text_box.color.r = 0.0f;
      text_box.color.g = 1.0f;
      text_box.color.b = 0.0f;
      text_box.color.a = 1.0f;
      text_box.text = std::to_string(id);
      text_box.lifetime = rclcpp::Duration(0.1);
      boxset_msgs.markers.push_back(text_box);
    }
    // }

    /**
     * 若该类别点云其整体的bounding box过大，则需网格化为smaller bounding
     box
     *
     */
    // else {
    //   /**
    //    * GridPoint_x：smaller bounding box中心的X轴坐标
    //    * GridPoint_y：smaller bounding box中心的Y轴坐标
    //    * 根据实际情况制定smaller bounding
    //    * box的长宽大小，主要依据是规避碰撞的预估距离
    //    * X和Y轴两个方向分别遍历
    //    */

    //   float GridPoint_x = min_x + 0.3;
    //   float GridPoint_y = min_y + 0.3;
    //   while (GridPoint_x < max_x) {
    //     while (GridPoint_y < max_y) {
    //       float distance_x = 0;
    //       float distance_y = 0;
    //       for (size_t j = 0; j < fitting_points[i].size(); j++) {
    //         distance_x = abs(fitting_points[i][j].x - GridPoint_x);
    //         distance_y = abs(fitting_points[i][j].y - GridPoint_y);
    //         if (distance_x < 0.3 && distance_y < 0.3) {
    //           GridBox.pose.position.x = GridPoint_x;
    //           GridBox.pose.position.y = GridPoint_y;
    //           GridBox.pose.position.z = min_z + heigth / 2;
    //           GridBox.scale.x = 0.6;
    //           GridBox.scale.y = 0.6;
    //           GridBox.scale.z = ((heigth < 0) ? -1 * heigth : heigth);
    //           GridBox.id = id;
    //           boxset_msgs.markers.push_back(GridBox);
    //           break;
    //         } else {
    //           continue;
    //         }
    //       }
    //       GridPoint_y = GridPoint_y + 0.6;
    //     }
    //     GridPoint_x = GridPoint_x + 0.6;
    //     GridPoint_y = min_y + 0.3;
    //   }
    // }
  }
  // LOG(ERROR) << "there are " << boxset_msgs.markers.size() << " boxsets "
  //            << std::endl;
  for (size_t i = 0; i < 100; i++) {
    if (isElementInVector(v_index, i)) {
      continue;
    }

    visualization_msgs::msg::Marker temp_box;
    visualization_msgs::msg::Marker text_box;
    visualization_msgs::msg::Marker trajectory_box;

    temp_box.header.frame_id = "camera";
    temp_box.ns = "box";
    temp_box.action = visualization_msgs::msg::Marker::DELETE;
    temp_box.id = i;
    boxset_msgs.markers.push_back(temp_box);

    text_box.header.frame_id = "camera";
    text_box.ns = "id";
    text_box.id = i;
    text_box.action = visualization_msgs::msg::Marker::DELETE;
    boxset_msgs.markers.push_back(text_box);

    trajectory_box.header.frame_id = "camera";
    trajectory_box.ns = "trajectory";
    trajectory_box.id = i;
    trajectory_box.action = visualization_msgs::msg::Marker::DELETE;
    boxset_msgs.markers.push_back(trajectory_box);
  }
}
}  // namespace CVTE_BABOT
