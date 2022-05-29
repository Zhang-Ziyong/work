/*
 * @Author: linyanlong
 * @Date: 2021-08-19 11:16:53
 * @LastEditTime: 2021-08-20 14:10:53
 * @LastEditors: Please set LastEditors
 * @Description: 绕障决策类单例，目前仅用于目标点选择使用
 * @FilePath:
 * /navigation/navigation/planner_decision/include/planner_decision.hpp
 */

#ifndef PLANNER_DECISION_HPP_
#define PLANNER_DECISION_HPP_
#include "path_follower.hpp"
#include "planner_decision_config.hpp"
namespace CVTE_BABOT {
class PlannerDecision {
 public:
  static std::shared_ptr<PlannerDecision> getInstance();
  /**
   *getTargetPoint
   *@brief
   *  获取最优的绕障目标点，根据输入的state进行判断，调用不同的绕障目标点选择函数
   **/
  bool getTargetPoint(const Pose2d &robot_pose, PathFollower *pf_ptr,
                      const FollowerStateRes &state, PoseMsg &target_pose_msg);
  /**
   *inputReferPath
   *@brief
   *  输入参考轨迹
   **/
  void inputReferPath(std::shared_ptr<SubPath> ptr_refer_path);
  /**
   *setConfig
   *@brief
   *  设置配置文件
   **/
  void setConfig(const PlannerDecisionConfig &config);

 private:
  PlannerDecision() {
    ptr_refer_path_ = nullptr;
    ptr_costmap_ = nullptr;
  }
  PlannerDecision(const PlannerDecision &obj) = delete;
  PlannerDecision &operator=(const PlannerDecision &obj) = delete;

  static std::shared_ptr<PlannerDecision> ptr_planner_decision_;

  int checkPointCostValue(const Pose2d &check_point,
                          std::shared_ptr<Costmap2d> ptr_costmap_2d);
  /**
   *getAvoiderTargetPoint
   *@brief
   *  获取refer过程中的期望目标点
   **/
  bool getAvoiderTargetPoint(PathFollower *pf_ptr, PoseMsg &target_pose_msg);
  /**
   *getBetterTargetPoint
   *@brief
   *  获取local过程中的更优目标点
   **/
  bool getBetterTargetPoint(PathFollower *pf_ptr, PoseMsg &target_pose_msg);
  /**
   *getRecoveryTargetPoint
   *@brief
   *  获取停障恢复的期望目标点
   **/
  bool getRecoveryTargetPoint(PathFollower *pf_ptr, PoseMsg &target_pose_msg);
  /**
   *getRecoveryTargetPoint
   *@brief
   *  获取过坎后的期望目标点
   **/
  bool getPitTargetPoint(PathFollower *pf_ptr, PoseMsg &target_pose_msg);

  /**
   * @brief referpath在地图内找不到目标点时,寻找一个附近没有被占用的点作为目标点
   *
   * @param[] target_pose -
   */
  bool refineClearPoint(PoseMsg &target_pose);
  /**
   *getSafeFactor
   *@brief
   *  获取安全性指标
   **/
  double getSafeFactor(const Pose2d &target_pose);
  /**
   *getBussinessFactor
   *@brief
   *  获取业务性指标
   **/
  double getBussinessFactor(double target_dist);
  /**
   *getBussinessFactor
   *@brief
   *  获取运动学指标
   **/
  double getDynamicFactor(const Pose2d &target_pose);
  /**
   *getMissionFactor
   *@brief
   *  获取任务指标
   **/

  double getMissionFactor(double length);

  struct PoseWithFactor {
    PoseMsg pose;
    double factor;
  };
  std::mutex refer_path_mutex_;
  std::mutex pose_mutex_;
  std::shared_ptr<SubPath> ptr_refer_path_;
  std::shared_ptr<Costmap2d> ptr_costmap_;
  Pose2d robot_pose_;
  PlannerDecisionConfig config_;
};
}  // namespace CVTE_BABOT
#endif