#ifndef _MSF_DATATYPE_H_
#define _MSF_DATATYPE_H_

#include <cmath>

#include "common/math.hpp"
#include "common/point_cloud.hpp"

namespace slam2d_core {
namespace msf {

class Pose2d {
 public:
  /**
   * @operator 构造函数
   * @brief 2d Pose类的构造函数
   */
  Pose2d() {
    this->x = 0;
    this->y = 0;
    this->yaw = 0;
  }

  Pose2d(const double x, const double y, const double yaw) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
  }

  ~Pose2d() {}
  /**
   * @operator *
   * @brief 2d Pose类的乘法运算
   * @param[in] 另一个pose1
   * @return 此pose经过pose1转换后的结果
   */
  Pose2d operator*(const Pose2d &pose1) const {
    Pose2d pose;
    pose.x =
        this->x + std::cos(this->yaw) * pose1.x - std::sin(this->yaw) * pose1.y;
    pose.y =
        this->y + std::sin(this->yaw) * pose1.x + std::cos(this->yaw) * pose1.y;
    pose.yaw = common::normalize_angle(this->yaw + pose1.yaw);
    return pose;
  }

  Pose2d inverse() const {
    Pose2d pose;
    pose.x = -this->x * std::cos(this->yaw) - this->y * std::sin(this->yaw);
    pose.y = this->x * std::sin(this->yaw) - this->y * std::cos(this->yaw);
    pose.yaw = common::normalize_angle(this->yaw * (-1.0));
    return pose;
  }

  /**
   * @operator -
   * @brief 2d Pose类的减法运算
   * @param[in] 另一个pose1
   * @return 此pose与pose1的差
   */
  // Pose2d operator-(const Pose2d &pose1) const {
  //   double dx = this->x - pose1.x;
  //   double dy = this->y - pose1.y;

  //   double delta_trans = std::sqrt(dx * dx + dy * dy);
  //   double delta_rot = common::angleDiff(this->yaw, pose1.yaw);
  //   double delta_bearing = common::angleDiff(std::atan2(dy, dx), pose1.yaw);

  //   Pose2d detla_pose;
  //   detla_pose.x = delta_trans * std::cos(delta_bearing);
  //   detla_pose.y = delta_trans * std::sin(delta_bearing);
  //   detla_pose.yaw = delta_rot;

  //   return detla_pose;
  // }
  /**
   * @operator ==
   * @brief 2d Pose类的等于判断运算
   * @param[in] pose-比较的pose
   * @return true-相同， false-不同
   */
  bool operator==(const Pose2d &pose) const {
    if (std::fabs(this->x - pose.x) > 1e-6) {
      return false;
    }
    if (std::fabs(this->y - pose.y) > 1e-6) {
      return false;
    }
    if (std::fabs(this->yaw - pose.yaw) > 1e-6) {
      return false;
    }
    return true;
  }

  double x;    ///< x轴变量
  double y;    ///< y轴变量
  double yaw;  ///< 航向角
};

class MSFNode {
 public:
  MSFNode() { cov.setZero(); }
  ~MSFNode() {}

  size_t node_id = 0;                ///< 节点编号
  Eigen::Matrix3d cov;               ///< 节点协方差
  common::Time time;                 ///< 节点时间
  Pose2d global_pose;                ///< 节点状态
  bool constrain_connected = false;  ///< 节点是否与出预测边外的边链接

  std::shared_ptr<common::PointCloud> pc_ptr =
      nullptr;  //< 对应一元边的激光点云(激光坐标系下)

  inline bool operator<(const MSFNode &n) const { return this->time < n.time; }
};

class UnaryConstrainData {
 public:
  UnaryConstrainData() { cov.setZero(); }
  ~UnaryConstrainData() {}

  common::Time time;    ///< 一元边对应时间
  Pose2d global_pose;   ///< 一元边数据
  Eigen::Matrix3d cov;  ///< 一元边协方差

  std::shared_ptr<common::PointCloud>
      pc_ptr;  //< 对应一元边的激光点云(激光坐标系下)

  inline bool operator<(const UnaryConstrainData &uc) const {
    return this->time < uc.time;
  }
};

class UnaryConstrain {
 public:
  UnaryConstrain() { cov.setZero(); }
  ~UnaryConstrain() {}

  size_t node_id;       ///< 一元边所链接的节点编号
  Eigen::Matrix3d cov;  ///< 一元边协方差
  common::Time time;    ///< 一元边对应时间
  Pose2d global_pose;   ///< 一元边数据

  inline bool operator<(const UnaryConstrain &uc) const {
    return this->time < uc.time;
  }
};

class BinaryConstrain {
 public:
  BinaryConstrain() { cov.setZero(); }
  ~BinaryConstrain() {}

  inline bool operator<(const BinaryConstrain &bc) const {
    return this->end_time < bc.end_time;
  }

  // inline common::Time relativeTime() const {
  //   return this->end_time - this->begin_time;
  // }

  size_t begin_node_id;     ///< 二元边前一段所链接的节点编号
  size_t end_node_id;       ///< 二元边后一段所链接的节点编号
  common::Time begin_time;  ///< 二元边开始时间
  common::Time end_time;    ///< 二元边结束时间
  Pose2d relative_pose;     ///< 二元边表示的相对位置
  Eigen::Matrix3d cov;      ///< 二元边的协方差
};

}  // namespace msf
}  // namespace slam2d_core

#endif