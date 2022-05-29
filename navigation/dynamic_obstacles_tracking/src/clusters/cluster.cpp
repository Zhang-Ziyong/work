#include <glog/logging.h>

#include "clusters/cluster.hpp"

namespace CVTE_BABOT {
void PointCloudCluster::updateParams(const ClusterParams &cp) {
  cluster_params_ = cp;
}

void PointCloudCluster::setInputCloud(
    const pcl::PointCloud<VPoint>::Ptr &ptr_input_pointcloud) {
  ptr_point_cloud_ = ptr_input_pointcloud;
}

void PointCloudCluster::recursiveClustering(
    const pcl::KdTreeFLANN<VPoint2D> &kdtree2d, const int &label_name,
    const VPoint2D &search_point, const float &radius,
    std::vector<int> &vi_label_index, std::vector<int> &vi_cluster_indexs) {
  std::vector<int> vi_point_index_in_radius;  // 点P的邻域之内的点的序列集
  vi_point_index_in_radius.reserve(50);
  std::vector<float> vf_squared_distance;  // 邻域之内的点到点P的距离
  vf_squared_distance.reserve(50);

  double used_radius = radius;
  // 过近不需要加
  if ((search_point.x * search_point.x + search_point.y * search_point.y) >
      5.0 * 5.0) {
    used_radius += cluster_params_.increment_radius;
    // 乗上激光本身的分辨率,弧度乗半径等于弧长约等于点之间的距离
    // used_radius +=
    //     cluster_params_.increment_radius *
    //     sqrt(search_point.x * search_point.x + search_point.y *
    //     search_point.y);
  }

  kdtree2d.radiusSearch(search_point, used_radius, vi_point_index_in_radius,
                        vf_squared_distance);

  for (size_t i = 0; i < vi_point_index_in_radius.size(); i++) {
    int neighbour_index = vi_point_index_in_radius[i];
    // 若未被标记, 则归为label_name, 并继续搜索附近的点
    if (vi_label_index[neighbour_index] == -1) {
      vi_label_index[neighbour_index] = label_name;
      vi_cluster_indexs.push_back(neighbour_index);

      // 用原始的半径radius,不可用累加后的半径used_radius,不然半径会不断累加到无穷
      recursiveClustering(kdtree2d, label_name,
                          ptr_point_cloud_2d_->points[neighbour_index], radius,
                          vi_label_index, vi_cluster_indexs);
    }
  }
}

void PointCloudCluster::extractSuitedCluster(
    const std::vector<std::vector<int>> &vvi_cluster_indexs,
    std::vector<pcl::PointCloud<VPoint>::Ptr> &v_ptr_cluster_points) {
  v_ptr_cluster_points.reserve(vvi_cluster_indexs.size());

  for (size_t cluster_index = 0; cluster_index < vvi_cluster_indexs.size();
       cluster_index++) {
    auto cluster_size = vvi_cluster_indexs[cluster_index].size();

    if (cluster_size >= cluster_params_.min_cluster_size) {
      auto ptr_cluster_points =
          pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);

      // 提取出相同类别的点存在一起
      for (const auto &point_index : vvi_cluster_indexs[cluster_index]) {
        if (ptr_point_cloud_->points[point_index].z > 0.8) {
          continue;
        }

        ptr_cluster_points->points.push_back(
            ptr_point_cloud_->points[point_index]);
      }

      v_ptr_cluster_points.push_back(ptr_cluster_points);
    }
  }
}

std::vector<pcl::PointCloud<VPoint>::Ptr>
PointCloudCluster::computeClustersPointCloud() {
  // 压缩到二维空间聚类
  ptr_point_cloud_2d_ =
      pcl::PointCloud<VPoint2D>::Ptr(new pcl::PointCloud<VPoint2D>);
  ptr_point_cloud_2d_->points.resize(ptr_point_cloud_->points.size());
  for (size_t i = 0; i < ptr_point_cloud_->points.size(); i++) {
    ptr_point_cloud_2d_->points[i].x = ptr_point_cloud_->points[i].x;
    ptr_point_cloud_2d_->points[i].y = ptr_point_cloud_->points[i].y;
  }

  // 记录当前点云所有点的类别, 默认为-1表示该点未被标记
  std::vector<int> vi_label_index;
  vi_label_index.resize(ptr_point_cloud_2d_->size(), -1);

  // 记录所有类包含的点云数量, 大小为MAX_LABELS_NUM
  std::vector<std::vector<int>> vvi_cluster_indexs;
  vvi_cluster_indexs.reserve(MAX_LABELS_NUM);

  // 为待处理的点云创建KD树的搜索方式
  pcl::KdTreeFLANN<VPoint2D> kdtree2d;
  kdtree2d.setInputCloud(ptr_point_cloud_2d_);

  int label_name = 0;

  for (size_t point_it = 0; point_it < ptr_point_cloud_2d_->size();
       point_it++) {
    // -1代表未被检索过
    if (vi_label_index[point_it] == -1) {
      vvi_cluster_indexs.push_back(std::vector<int>());
      vvi_cluster_indexs.back().reserve(100);

      recursiveClustering(kdtree2d, label_name,
                          ptr_point_cloud_2d_->points[point_it],
                          cluster_params_.cluster_radius, vi_label_index,
                          vvi_cluster_indexs.back());

      assert(!vvi_cluster_indexs.back().empty());
    }
    label_name++;
  }

  std::vector<pcl::PointCloud<VPoint>::Ptr> v_ptr_cluster_points;
  extractSuitedCluster(vvi_cluster_indexs, v_ptr_cluster_points);

  return v_ptr_cluster_points;
  // v_last_valid_index_.swap(v_valid_index);
}

}  // namespace CVTE_BABOT
