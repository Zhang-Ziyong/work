/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file path_manager.hpp
 *
 *@brief 管理路径的类，包括 任务路径、局部路径以及控制路径等。
 *       通过本类对上述路径的设置、获取和更新，提供相应接口
 *       ---------------------------------------
 *       局部路径规划的算法，放在这个类里面使用，外面只需触发
 *       更新局部路径的接口就行，不需用什么算法
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 06
 ************************************************************************/

#ifndef __PATH_MANAGER_HPP
#define __PATH_MANAGER_HPP

#include <memory>
#include <mutex>
#include "local_planner.hpp"
#include "path.hpp"

namespace CVTE_BABOT {

class LocalPlanner;

class PathManager {
 public:
  PathManager() = default;
  ~PathManager() = default;

  PathManager(const PathManager &) = delete;
  PathManager &operator=(const PathManager &) = delete;

  /**
   *PathManager
   *@brief
   *  构造函数
   *
   *@param[in] costmap - 代价地图
   *@param[in] local_algo - 内部使用的局部规划算法名称
   **/
  PathManager(std::shared_ptr<Costmap2d> costmap,
              const std::string &local_algo);

  // /**
  //  *setReferPath
  //  *@brief
  //  *  设置任务路径，此任务路径是通过任务管理下发的
  //  *  一般是两个巡逻点之间的线段
  //  *
  //  *@param[in] refer_path - 需要执行的任务路径
  //  **/
  // void setReferPath(const SubPath &refer_path);

  /**
   *@brief
   *  1、getLocalPath - 获取局部路径
   *  2、getReferPath - 获取任务路径
   *  3、getCtrolPath - 获取控制路径
   *
   *@return 对应的路径数组
   **/
  std::shared_ptr<SubPath> getLocalPath();
  std::shared_ptr<SubPath> getReferPath();
  std::shared_ptr<SubPath> getCtrolPath();

  /**
   *@brief
   *  1、setLocalPath - 设置局部路径
   *  2、setReferPath - 设置任务路径
   *  3、setCtrolPath - 设置控制路径
   *
   **/
  void setLocalPath(std::shared_ptr<SubPath> path);
  void setReferPath(std::shared_ptr<SubPath> path);
  void setCtrolPath(std::shared_ptr<SubPath> path);

  /**
   *@brief
   *  1、isLocalPathEmpty - 判断局部路径是否为空
   *  2、isReferPathEmpty - 判断任务路径是否为空
   *  3、isCtrolPathEmpty - 判断控制路径是否为空
   *
   *@return true - 代表该路径为空， 否则相反
   **/
  bool isLocalPathEmpty();
  bool isReferPathEmpty();
  bool isCtrolPathEmpty();

  /**
   * updateLocalPath 与 多态版本，支持指定起点和终点来更新局部路径
   * 或者只指定目标点，当能规划成功后同时更新此时绕障目标点
   *@brief
   *  重新规划一次局部路径
   *  规划的目标点为local_target_point_，在内部保存
   *  这个目标点的修改一般在监督器内被修改
   *
   *@return true - 代表该路径为空， 否则相反
   **/
  bool updateLocalPath();
  bool updateLocalPath(const PoseMsg &target);
  bool updateLocalPath(const PoseMsg &start, const PoseMsg &target);

  /**
   * updatePassPitPath，指定起点、终点、坎起点，坎终点来更新局部路径
   *@brief
   *  重新规划一次局部路径
   *  规划的目标点为local_target_point_，在内部保存
   *  这个目标点的修改一般在监督器内被修改
   *
   *@return true - 代表该路径为空， 否则相反
   **/
  bool updatePitPath(const Pose2d &pit_in_pose, const Pose2d &pit_out_pose,
                     const PoseMsg &target);
  /**
   *@brief
   *将局部路径上的部分姿态和速度信息取反
   *为了能够保证机器人倒着跟轨迹
   *@return
   **/
  void reverseLocalPath(int sidx, int eidx);

  /**
   *@brief
   *将参考路径上的部分姿态和速度信息取反
   *为了能够保证机器人倒着跟轨迹
   *@return
   **/
  void reverseReferPath(int sidx, int eidx);

  /**
   * cycleUpdateLocalPath
   *@brief
   *  循环更新局部路径，为了避免由于左右路径长度相近而来回摆动的问题
   *  添加了逻辑判断：
   *     两次之间的规划的长度变化率低于一定值时，不更新到local_path
   *
   *@return true - 代表该路径为空， 否则相反
   **/
  bool cycleUpdateLocalPath();

  /**
   * updateCtrolPath
   *@brief
   *  更新控制路径，一般是根据当前状态，从局部路径或任务路径中截取
   *  控制路径的作用是直接给局部控制器做路径跟踪使用
   *
   *@param[in] from - 从局部或者从任务路径中截取（"local"或"refer"）
   *@return true - 代表更新成功，否则相反
   **/

  bool updateCtrolPath(const std::string &from);

  /**
   * setLocalTarget
   *@brief
   *  设置局部路径目标点，监督器会根据监督结果，重新赋值到这里
   *  在path_follower中无须关心当前目标点是什么
   *
   *@return true - 代表更新成功，否则相反
   **/
  void setLocalTarget(const PoseMsg &point);

  /**
   * getLocalTarget
   *@brief
   *  获取当前局部路径的目标点
   *
   *@return 此时监督器更新的目标点
   **/
  inline PoseMsg getLocalTarget();

  /**
   * resetLocalTarget
   *@brief
   *  重置当前绕障目标点信息，避免找不到目标点时前往0，0，0
   *
   **/
  void resetLocalTarget();

  /**
   *updateCostmap
   *@brief
   *  更新代价地图
   *
   *@param[in] costmap - 当前环境信息下的代价地图信息
   **/
  inline void updateCostmap(std::shared_ptr<Costmap2d> costmap) {
    ptr_local_costmap_ = costmap;
    ptr_costmap_2d_ = costmap;
    ptr_local_planner_->updateCostMap(costmap);
    ptr_dijk_planner_->updateCostMap(costmap);
  }

  /**
   *updateGlobalCostmap
   *@brief
   *  更新全局代价地图
   *
   *@param[in] costmap - 当前环境信息下的全局代价地图信息
   **/
  inline void updateGlobalCostmap(std::shared_ptr<Costmap2d> costmap) {
    ptr_global_costmap_ = costmap;
  }

  /**
   *getCostmap
   *@brief
   *  获取当前代价地图
   *
   *@return 代价地图的指针
   **/
  inline std::shared_ptr<Costmap2d> getCostmap() { return ptr_costmap_2d_; }

  /**
   *setMaxDistance
   *@brief
   *  设置允许机器人偏离控制路径的最大路径
   *
   *@param[in] distance - 最大直线距离（单位：米）
   **/
  inline void setMaxDistance(const double &distance) {
    max_distance_ = distance;
  }

  /**
   *setOptimizeLocalPathFlag、setOptimizeLocalParams、setOptCtrlPointsParams
   *@brief
   *  设置路径优化的相关参数
   **/
  void setOptimizeLocalPathFlag(bool optimize_flag, bool vel_limit_ctrl,
                                bool optimize_ctrl);
  void setOptimizeLocalParams(double v_limit, double w_limit, double acc_limit,
                              double sample_delta_time, double bsp_delta_time);
  void setOptCtrlPointsParams(double lambda1, double lambda2, double lambda3,
                              double lambda4, bool use_endpt_opt, double dist,
                              double min_rot_radiu, int max_iter_num,
                              double max_iter_time);
  void setPitParams(double edge_dist, double access_interval) {
    pit_edge_dist_ = edge_dist;
    pit_access_interval_ = access_interval;
  }

  // /**
  //  *isNewReferPath
  //  *@brief
  //  *  判断是否一条新的任务路径
  //  *
  //  *@return true - 新路径，否则相反
  //  **/
  // bool isNewReferPath();

  // /**
  //  *clearNewReferPathFlag
  //  *@brief
  //  *  清除新路径的标志位
  //  *
  //  **/
  // void clearNewReferPathFlag();

  /**
   *getFullCtrolPath
   *@brief
   *  获取控制路径指针
   **/
  SubPath getFullCtrolPath();

  /**
   *getXXXXPathAngle
   *@brief
   *  获取当前路径角度
   **/
  double getCtrolPathAngle();
  double getReferPathAngle();

  // /**
  //  *setLocalPath
  //  *@brief
  //  *  将规划的结果赋值给局部路径
  //  **/
  // void setLocalPath(const SubPath &path);
  /**
   *isNeedUpdateReferCtrlPath
   *@brief
   *  是否需要更新任务控制路径
   *
   **/
  bool isNeedUpdateReferCtrlPath();
  /**
   *isNeedUpdateLocalCtrlPath
   *@brief
   *  是否需要更新局部绕障路径
   *
   **/
  bool isNeedUpdateLocalCtrlPath();

  /**
   *makeStraightLinePath
   *@brief
   *  两点之间规划直线路径
   *
   **/
  bool makeStraightLinePath(const Pose2d start, const Pose2d end,
                            SubPath &path);

  /**
   *checkPointCostValue
   *@brief
   *  检查输入的点是否发生碰撞
   *
   **/
  bool checkPointCostValue(const Pose2d &check_point);

  /**
   *checkObstacleBetweenTwoPoints
   *@brief
   *  判断两点之间是否存在障碍物
   *
   **/
  bool checkObstacleBetweenTwoPoints(const Pose2d start, const Pose2d target);

  bool makeLocalPlan(const Pose2d &start, const Pose2d &end,
                     const double &start_vel, const double &end_vel,
                     SubPath &result);

  bool makeDiskPlan(const Pose2d &start, const Pose2d &end,
                    const double &start_vel, const double &end_vel,
                    SubPath &result);

  std::vector<Pose2d> getOriginalLocalPath() { return original_local_path_; }

  SubPath optimizeLocalPath(const SubPath &path_pose, const double &start_vel,
                            const double &end_vel);

  /**
   *getReferLastPoint
   *@brief
   *  获取路径任务路径的最后一点坐标值
   *
   **/
  Pose2d getReferLastPoint();

  /**
   *与路径索引值的设置、更新、查询等相关接口
   *@brief
   *  setReferIndex —— 设置当前总任务路径的索引
   *  updateReferIndex ——
   *将当前总任务路径的索引值更新到绕障目标点，在绕障完成时调用
   *  getReferIndex —— 获取此时总路径的索引值
   *  getTargetIndex —— 获取此时绕障目标点在总路径上的索引值
   *  calcReferIndexWithDistance ——
   *在总任务路径上，根据当前索引值开始算dis米后的索引值，若超出剩余距离则返回路径最后一点索引
   *
   **/
  inline void setReferIndex(size_t index);
  void setLocalIndex(size_t index);
  void setAvoiderReferIndex(size_t index);
  void updateReferIndex();
  size_t getReferIndex();
  size_t getLocalIndex();
  size_t getAvoiderReferIndex();
  size_t getTargetIndex();
  size_t getReferPathSize();
  size_t getLocalPathSize();
  size_t calcReferIndexWithDistance(const double &dis);

  // /**
  //  *getReferRemainLength
  //  *@brief
  //  *  返回从当前跟踪的索引开始，到路径末端剩余的距离长度
  //  *  针对任务路径
  //  * return 剩余任务路径长度（米）
  //  **/
  // double getReferRemainLength();

  /**
   *setForwardStatus
   *@brief
   *  设置规划状态为前进或后退
   * return 无
   **/
  inline void setForwardStatus(const bool forward) { forward_ = forward; }
  /**
   *getCompletion
   *@brief
   *  返回从跟踪跟踪任务完成度
   *  针对任务路径
   * return 路径跟踪完成度
   **/
  double getCompletion();

 private:
  void resetLocalIndex();

  ///< 两点之间的路径规划算法，用于更新局部路径
  std::shared_ptr<LocalPlanner> ptr_local_planner_ = nullptr;
  std::shared_ptr<LocalPlanner> ptr_dijk_planner_ = nullptr;

  ///< 局部路径，用于避让车辆行人等障碍时使用的路径
  std::shared_ptr<SubPath> ptr_local_path_ = nullptr;
  ///< 任务路径，上层发送下来需要跟踪的路径
  std::shared_ptr<SubPath> ptr_refer_path_ = nullptr;
  ///< 控制路径，发给local_controller执行的路径（从局部路径或任务路径中截取）
  std::shared_ptr<SubPath> ptr_ctrol_path_ = nullptr;

  // 说明: 当前使用代价地图默认是使用局部代价地图，但有时局部代价地图太小，
  // 导致绕障轨迹规划不出来，此时将当前使用代价地图切换为全局代价地图和局部代价地图的结合
  std::shared_ptr<Costmap2d> ptr_costmap_2d_ = nullptr;  ///< 当前使用代价地图
  std::shared_ptr<Costmap2d> ptr_local_costmap_ = nullptr;  ///< 局部代价地图
  std::shared_ptr<Costmap2d> ptr_global_costmap_ = nullptr;  ///< 全部代价地图

  std::mutex refer_target_index_mutex_;  ///< 目标点索引的锁
  std::mutex target_mutex_;              ///< 目标点锁
  std::mutex local_path_mutex_;          ///< 局部路径锁
  std::mutex refer_path_mutex_;          ///< 全局路径锁
  std::mutex ctrol_path_mutex_;          ///< 控制路径锁
  std::mutex path_direct_mutex_;
  std::mutex avoider_refer_index_mutex_;
  PoseMsg local_target_point_;  ///< 保存局部路径的目标点，监督器会更改这个点
  double max_distance_ = 0.0;  ///< 允许机器人位置到控制路径的最大直线距离
  std::string local_planner_algorithm_;

  bool optimize_local_path_ = true;  /// 是否需要对局部路径进行优化
  bool calc_vel_limit_ctrl_ = true;
  bool calc_optimize_ctrl_ = true;
  bool refer_path_update_flag_ = false;
  bool local_path_update_flag_ = false;
  double v_limit_ = 0.0;
  double w_limit_ = 0.0;
  double acc_limit_ = 0.0;
  double sample_delta_time_ = 0.0;
  double bsp_delta_time_ = 0.0;

  double lambda1_ = 0.0;
  double lambda2_ = 0.0;
  double lambda3_ = 0.0;
  double lambda4_ = 0.0;
  double dist_ = 0.0;
  double min_rot_radiu_ = 0.0;
  double max_iter_time_ = 0.0;
  int max_iter_nums_ = 0;
  bool use_endpt_opt_ = false;

  size_t refer_path_index_ = 0;
  size_t local_path_index_ = 0;
  size_t avoider_refer_path_index_ = 0;
  size_t local_path_size_ = 0;
  size_t refer_path_size_ = 0;
  bool return_to_referring_path_ = false;
  std::vector<Pose2d> original_local_path_;

  bool forward_ = true;

  double pit_edge_dist_ = 0;        ///< 过坎点与坎边界的间距
  double pit_access_interval_ = 0;  ///< 过坎延边距离
};

}  // namespace CVTE_BABOT

#endif  // end of __PATH_MANAGER_HPP
