#ifndef _RBNN_CLUSTER_H_
#define _RBNN_CLUSTER_H_

#include <vector>

namespace slam2d_core {
namespace amcl {

typedef struct {
  double x;
  double y;
} rbnn_point_t;

/**
 * RBNN
 * @brief 利用RBNN算法对2维点进行聚类
 **/
class RBNN {
 public:
  /**
   * cluster
   * @brief 利用RBNN算法对2维点进行聚类
   * @param[in] pl-二维点集
   * @param[in] clusters-返回类别个数
   * @param[in] lbs-返回标签集合
   * @param[in] r-聚类最大距离
   * @param[in] min_cluster_num-每个有效类中最小的点数
   * @param[in] ANGLE_INC-激光角度分辨率
   **/
  void cluster(const std::vector<rbnn_point_t> &pl, int &clusters,
               std::vector<int> &lbs, const double &r,
               const int &min_cluster_num, const double &ANGLE_INC);

 private:
  /**
   * mergeCluster
   * @brief 合并两种标签
   * @param[in] lbs-标签集合
   * @param[in] l1-标签1
   * @param[in] l2-标签2
   **/
  void mergeCluster(std::vector<int> &lbs, const int &l1, const int &l2);

  /**
   * findNeighbors
   * @brief 找到一个点附近的点
   * @param[in] idx-目标点索引
   * @param[in] pl-标签集合
   * @param[in] r-聚类最大距离
   * @param[in] nbn-从邻居几个点进行搜索
   * @param[in] ANGLE_INC-激光角度分辨率
   * @return 相邻点索引
   **/
  unsigned int findNeighbors(std::vector<int> &nidx, const unsigned int &idx,
                             const std::vector<rbnn_point_t> &pl,
                             const double &r, const unsigned int &nbn,
                             const double &ANGLE_INC);
};

}  // namespace amcl
}  // namespace slam2d_core
#endif