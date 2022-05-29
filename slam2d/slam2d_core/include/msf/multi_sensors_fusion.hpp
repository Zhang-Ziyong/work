#ifndef _MULTI_SENSORS_FUSION_H_
#define _MULTI_SENSORS_FUSION_H_

#include <atomic>
#include <list>
#include <mutex>
#include <string>

#include "Eigen/Core"
#include "common/time.hpp"
#include "msf_datatype.hpp"
#include "pose_factor.hpp"

namespace slam2d_core {
namespace msf {

class MultiSensorsFusionOptions {
 public:
  bool do_node_merge = true;   ///< 是否进行节点合并
  bool fix_first_node = true;  ///< 在优化过程中是否固定第一个节点
  bool do_time_synchronization = true;  ///< 是否进行时间同步
  bool do_time_check = false;           ///< 是否检查数据时间
  bool compute_cov = true;              ///< 是否计算协方差

  size_t WINDOW_SIZE = 50;           ///< 维护的最大优化节点数
  int bad_match_node_number_ = 200;  //< 因子图连续差匹配时的节点数量阈值
  bool use_laserodom_constrain = true;  //< 判断是否添加雷达里程计约束
};

class MultiSensorsFusion {
 public:
  explicit MultiSensorsFusion(const MultiSensorsFusionOptions &options);
  MultiSensorsFusion &operator=(const MultiSensorsFusion &obj) = delete;
  ~MultiSensorsFusion();

  void setInitializeNode(const MSFNode &node);
  void getInitializeNode(MSFNode &node) { node = init_node_; }
  bool newAmclUnaryConstrain(const double &huber_loss);

  bool addPredictedConstrainData(const UnaryConstrainData &odom_new_data,
                                 const UnaryConstrainData &laserodom_new_data);
  bool addAmclUnaryConstrainData(const UnaryConstrainData &new_data);

  void clear();
  bool build();
  bool optimize();
  bool getLocalizationResult(Pose2d &locatization_result,
                             common::Time &locatization_time);
  bool getUnaryNodes(std::vector<MSFNode> &last_unary_nodes);

  bool getCov(double &xx, double &yy, double &aa);
  std::string graphInfo(std::string info_name);
  size_t getPredictedSize();
  bool initialization();

  enum STATUS {
    UNINITIALIZED = 0,      ///< 未初始化
    WORKING_NORMALLY,       ///< 正常运行
    LOCALIZATION_UNSTABLE,  ///< 结果不可靠
    LOCALIZATION_ERROR      ///< 发生错误
  };

 private:
  const std::list<MSFNode>::iterator findNode(const size_t node_id);
  const std::list<MSFNode>::iterator findLaserodomNode(const size_t node_id);

  bool buildUnaryConstrain(
      const std::shared_ptr<std::list<UnaryConstrain>> &constrains,
      const std::shared_ptr<std::list<UnaryConstrainData>> &buffer);
  bool buildLaserodomNodes(
      const std::shared_ptr<std::list<UnaryConstrainData>> &buffer);

  Eigen::Matrix3d computeBinaryConstrainCov(
      const std::list<MSFNode>::iterator node1,
      const std::list<MSFNode>::iterator node2);

  void addBinaryResidualBlock(
      ceres::Problem &problem, ceres::LossFunction *loss_function,
      ceres::LocalParameterization *alp,
      const std::list<BinaryConstrain> &binary_constrains);

  void addUnaryResidualBlock(ceres::Problem &problem,
                             ceres::LossFunction *loss_function,
                             ceres::LocalParameterization *alp,
                             const std::list<UnaryConstrain> &unary_constrains);

  MSFNode init_node_;  ///< 初始节点
  MultiSensorsFusionOptions options_;
  std::recursive_mutex init_pose_mutex_;  ///< 初始节点数据的互斥锁

  size_t node_id_ = 0;                    ///< 节点变化
  size_t new_node_index_ = 0;             ///< 图中新加入节点的索引
  std::list<MSFNode> nodes_;              ///< 图节点
  std::recursive_mutex graph_mutex_;      ///< 图操作互斥锁
  std::recursive_mutex predicted_mutex_;  ///< 预测数据缓存数据的互斥锁
  std::list<UnaryConstrainData> predicted_buffer_;   ///< 预测数据缓存
  std::list<BinaryConstrain> predicted_constrains_;  ///< 预测数据边
  UnaryConstrainData last_predicted_data_;  ///< 上一个插入的预测数据

  size_t laserodom_node_id_ = 0;         ///< 激光里程计节点号
  size_t new_laserodom_node_index_ = 0;  ///< 激光里程计buffer
  ///< 长度，现在激光里程计nodes 对应的下标
  std::list<MSFNode> laserodom_nodes_;  ///< 激光里程计图节点
  std::list<UnaryConstrainData> laserodom_buffer_;  ///< 激光里程计数据缓存
  std::list<BinaryConstrain> laserodom_constrains_;
  UnaryConstrainData last_laserodom_data_;  ///< 上一个激光里程计的预测数据

  std::shared_ptr<std::list<UnaryConstrain>> amcl_unary_point_constrain_ =
      nullptr;
  std::shared_ptr<std::list<UnaryConstrain>> amcl_unary_constrains_ = nullptr;
  std::shared_ptr<std::list<UnaryConstrainData>>
      amcl_unary_point_constrain_buffer_;  ///< 一元边数据缓存（无角度）
  std::shared_ptr<std::list<UnaryConstrainData>>
      amcl_unary_constrain_buffer_;              ///< 一元边数据缓存
  std::shared_ptr<std::recursive_mutex> mutex_;  ///< 边缓存数据的互斥锁
  double huber_loss_;  ///< 边的huber loss函数参数

  std::atomic<bool> initializing_;  ///< 算法初始话状态
  bool init_predicted_ = false;     ///< 预测数据是否已经初始化

  Pose2d last_optimize_pose_;        ///< 最近一次的优化结果
  common::Time last_optimize_time_;  ///< 最近一次优化的数据的时间
  double current_pose_cov_[3];       ///< 最近一次优化计算的协方差

  size_t status_ = 0;  ///< 算法运行状态
  size_t cnt_ = 0;

  Pose2d last_locatization_result_;  ///< 最近获取的定位结果
};

}  // namespace msf
}  // namespace slam2d_core
#endif
