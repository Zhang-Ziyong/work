/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file path_follower.hpp
 *
 *@brief 用于跟踪路径
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-18
 ************************************************************************/
#ifndef __PATH_FOLLOWER_HPP
#define __PATH_FOLLOWER_HPP

#include <chrono>
#include <mutex>

#include "local_controller_base.hpp"
#include "navigation_mediator.hpp"
#include "path_manager.hpp"
#include "planner_utils.hpp"
#include "robot_info.hpp"
#include "speed_controller.hpp"
#include "supervisor/supervisor_chain.hpp"
#include "planner_decision/planner_decision_config.hpp"
#include "local_controller_factory.hpp"
#include "state/refer_state.hpp"
#include "state/pause_state.hpp"
#include "speed_decision_base.hpp"

#define MCMDSTATE MoveCommand::MoveCommandStatus

namespace CVTE_BABOT {

class Costmap2d;
class PathManager;
class MoveCommand;
class LocalPlanner;
class BasePathRecovery;
class LocalController;

// ----- 状态类的前置声明
class State;
class FreeState;
class LocalState;
class ReferState;

// ----- 监督器类的前置声明
class AngleTopathSupervisor;
class DistanceToPathSupervisor;
class FinalGoalSupervisor;
class ObstacleAvoiderSupervisor;
class SpeedLevelSupervisor;
class OverTimeSupervisor;
class MarkerMapSupervisor;
class PassPitSupervisor;

class PathFollower {
 public:
  PathFollower(std::shared_ptr<Costmap2d> costmap);
  PathFollower() = delete;

  ~PathFollower() = default;

  /**
   *setTargetPose
   *@brief
   *设置任务终点 TODO_终点
   *
   *@param[in] target_pose - 需要导航跟踪的任务路径
   **/
  void setTargetPose(const Pose2d &target_pose);

  /**
   *setReferencePath
   *@brief
   *设置任务路径
   *
   *@param[in] reference_path - 需要导航跟踪的任务路径
   **/
  void setReferencePath(std::shared_ptr<SubPath> reference_path);

  /**
   *setRobotType
   *@brief
   *设置机器人类型,使用不同的策略
   *
   *@param[in] type - 机器人类型
   **/
  void setRobotType(ROBOTTYPE type);

  /**
   *setCurrentPose
   *@brief
   *更新定位信息，并更新给local_controller
   *
   *@param[in] current_pose - 机器人在世界坐标系下的定位信息
   **/
  void setCurrentPose(const Pose2d &current_pose);

  /**
   *setCurrentVelocity
   *@brief
   *设置当前速度
   *
   *@param[in] current_velocity -
   *机器人当前速度，从odom获取的，设置给Local_controller
   **/
  void setCurrentVelocity(const Velocity &current_velocity);

  /**
   *followerUpdate
   *@brief
   * 通过监督器，观察机器人的运行姿态。（包括行人检测、绕障检测、目标点占用等）切换不同的控制状态
   * 单独一个线程在更新控制状态，运行频率可通过navigation_ros2的参数文件设置，目前2Hz
   *
   *@param[out] FollowerStateRes -
   *返回机器人当前运行状态（REFER、LOCAL、PAUSE等）
   **/
  FollowerStateRes followerUpdate();

  /**
   *controllerExecute
   *@brief
   * local_controller的主要计算函数，
   *
   *@param[out] FollowerStateRes -
   *返回机器人当前运行状态（REFER、LOCAL、PAUSE等）
   **/
  PathFollowerStates controllerExecute(
      MoveCommand &move_command,
      MoveCommand::MoveCommandStatus &move_command_status);

  /**
   *updateXXXCostMap
   *@brief
   * 更新内部costmap
   *
   *@param[in] costmap - 当前最新的costmap函数指针
   **/
  inline void updatePlanerCostMap(std::shared_ptr<Costmap2d> costmap) {
    ptr_path_manager_->updateCostmap(costmap);
  }
  inline void updatePlanerGlobalCostMap(std::shared_ptr<Costmap2d> costmap) {
    ptr_path_manager_->updateGlobalCostmap(costmap);
  }
  inline void updateControllerCostMap(std::shared_ptr<Costmap2d> costmap) {
    ptr_local_controller_->updateCostMap(costmap);
  }

  /**
   *getPmPtr
   *@brief
   * 获取路径管理类的指针函数，在状态类中获取和修改路径使用
   **/
  std::shared_ptr<PathManager> getPmPtr() { return ptr_path_manager_; }

  /**
   *changeState
   *@brief
   *  修改当前状态，让状态指针指向新的单例状态
   *
   *@param[in] state_ptr - 目标状态的指针（单例）
   **/
  void changeState(const std::shared_ptr<State> state_ptr);

  /**
   *setPathManagerParams
   *@brief
   *  设置路径优化的参数
   **/
  void setPathManagerParams();

  /**
   *setRotateVelocity && getRotateVelocity
   *@brief
   *  设置和获取当前旋转的速度，速度值的计算在旋转状态中修改
   **/
  void setRotateVelocity(const double &velocity);
  double getRotateVelocity();
  /**
   *getControllerPathRemainLength
   *@brief
   *  获取控制器剩余path的路径长度
   **/
  double getControllerPathRemainLength();
  /**
   *getReferRemainLength
   *@brief
   *  获取到终点的路径总长度
   **/

  double getReferRemainLength();
  /**
   * @brief 获取轨迹跟踪索引
   *
   * @return size_t
   */
  size_t getReferIndex() const { return ptr_path_manager_->getReferIndex(); }

  /**
   *recoverClear
   *@brief
   *  清空恢复临时变量
   **/
  void recoverClear();

  /**
   *checkIfNeedToRecover
   *@brief
   *  判断是否需要后退，通过检测机器人前方是否有障碍物决定
   **/
  bool checkIfNeedToRecover();

  /**
   *getXXXXPath
   *@brief
   *  向外暴露获取各种路径点信息的接口，用于外部可视化使用
   **/
  std::vector<Pose2d> getOriginalLocalPath();
  std::vector<Pose2d> getCtrlPath();

  inline void setForwardStatus(const bool forward) {
    forward_ = forward;
    ptr_path_manager_->setForwardStatus(forward);
  }

  inline void setPathStatus(MISSIONTYPE mission_type) {
    mission_type_ = mission_type;
    LocalControllerFactory local_controller_factory;
    if (mission_type_ != MISSIONTYPE::EDGE) {
      ptr_local_controller_ = local_controller_factory.createLocalController(
          local_controller_algorithm_, ptr_costmap_2d_);
    } else {
      LOG(WARNING) << "edge contrler : " << edge_controller_algorithm_;
      ptr_local_controller_ = local_controller_factory.createLocalController(
          edge_controller_algorithm_, ptr_costmap_2d_);
    }
  }

  inline MISSIONTYPE getPathStatus() { return mission_type_; }
  inline void setFinalRotateFlag(bool need_final_rotate) {
    need_final_rotate_ = need_final_rotate;
    ReferState::getPtrInstance()->setFinalRotateFlag(need_final_rotate);
  }

  void setAbsReachFlag(bool is_abs_reach);

 private:
  /**
   *getState
   *@brief
   *  获取当前状态
   **/
  PathFollowerStates getState();

  /**
   *updateParams
   *@brief
   *  读取并更新参数
   **/
  void updateParams();

  /**
   *constructSupervisor
   *@brief
   *  构造监督器，添加不同的构造类
   **/
  void constructSupervisor(const unsigned int &ui_size_x,
                           const unsigned int &ui_size_y);

  /**
   *showMeDebugMessage
   *@brief
   *  调试使用，打印当前位置信息和速度信息
   *
   *@param[in] SpeedCtrlRes - 三个轴（XYZ）的最大线速度
   **/
  void showMeDebugMessage();

  /**
   *localControllerUpate
   *@brief
   *  通过Local_controller正常计算控制速度的函数
   *  不包含停障恢复的操作
   *
   *@param[out] move_command - 需要执行的控制速度（计算结果）
   *@param[out] move_command_status - 计算结果状态
   **/
  void localControllerUpate(MoveCommand &move_command,
                            MCMDSTATE &move_command_status);

  /**
   *setMoveCmd
   *@brief
   *  修改速度值
   *
   *@param[in] move_command - 需要设置的速度命令
   *@param[in] x - x轴速度
   *@param[in] y - y轴速度
   *@param[in] th - 角速度
   **/
  void setMoveCmd(MoveCommand &move_command, const double &x, const double &y,
                  const double &th);

  /**
   *errorZeroMoveCmd
   *@brief
   *  计算错误，发送零速度并设置错误状态
   *
   *@param[out] move_command - 需要设置的速度命令
   *@param[out] move_command_status - 控制命令状态
   **/
  void errorZeroMoveCmd(MoveCommand &move_command,
                        MCMDSTATE &move_command_status);

  /**
   *normalZeroMoveCmd, doneZeroMoveCmd
   *@brief
   *  正常停车指令(暂停或完成情况)
   *
   *@param[out] move_command - 需要设置的速度命令
   *@param[out] move_command_status - 控制命令状态
   **/
  void normalZeroMoveCmd(MoveCommand &move_command,
                         MCMDSTATE &move_command_status);
  void doneZeroMoveCmd(MoveCommand &move_command,
                       MCMDSTATE &move_command_status);

  /**
   *updateCtrlPath
   *@brief
   *  根据当前状态，更新控制路径
   *
   *@return true - 代表更新成功
   **/
  bool updateVisCtrlPath();

  /**
   *isTimeToUpdateCtrlPath
   *@brief
   *  更新控制路径的频率，因为目前系统控制路径的更新在控制线程，
   *  而更新控制路径不需要太高频，避免浪费资源
   *
   *@return true - 代表需要更新一次控制路径
   **/
  bool isTimeToUpdateVisCtrlPath();

  void processRotateCtrl(MoveCommand &move_command,
                         MCMDSTATE &move_command_status);
  void processRecoverCtrl(MoveCommand &move_command,
                          MCMDSTATE &move_command_status);
  void processFollowCtrl(MoveCommand &move_command,
                         MCMDSTATE &move_command_status);

  // -------------------------------------------------------
  // ===================== 分割线 ===========================
  // -------------------------------------------------------

  bool robot_can_rotate_ = true;  ///< 机器人是否支持原地旋转
  bool enable_recover_ = false;   ///< 是否启用后退功能,默认关
  bool enable_rotate_ = false;    ///< 是否启用旋转功能,默认关
  bool enable_pit_ = false;       ///< 是否启动过坎功能，默认关
  bool optimize_local_path_ = false;

  Pose2d global_target_pose_;
  std::string local_planner_algorithm_ = "";
  std::string local_controller_algorithm_ = "";
  std::string edge_controller_algorithm_ = "";

  std::mutex state_mutex_;
  std::mutex rotate_velocity_mutex_;
  PathFollowerStates mark_last_state_ =
      PathFollowerStates::FREE;  ///< controller线程记录状态的变化

  double max_angle_to_path_ = 1.0;
  double max_distance_to_path_ = 1.0;
  double obstacle_avoid_distance_ = 1.5;
  double obstacle_avoid_behind_ = 1.5;
  double d_reach_goal_tolerance_ = 0.3;
  double yaw_tolerance_ = 0.0;
  double xy_tolerance_ = 0.0;
  double max_recover_distance_ = 1.0;
  double rotate_velocity_ = 0.0;
  double min_velocity_threshold_ = 0.0;
  double min_rotate_threshold_ = 0.0;
  double planner_frequence_ = 0.0;
  double controller_frequence_ = 0.0;
  int update_ctrl_path_count_ = 0;
  int dangerous_value_ = 150;
  double over_time_wait_sec_ = 10.0;
  double pause_wait_sec_ = 60.0;
  double rotate_wait_sec_ = 5.0;
  double over_time_wait_dist_ = 2.0;
  double pit_avoid_distance_ = 0;  ///< 过坎点检索长度

  PlannerDecisionConfig planner_decision_config_;

  // 窄道导航策略监督器相关参数
  bool narrow_enabled_ = false;          // 是否启动窄道导航策略
  double path_dir_length_ = 1.0;         // 确定轨迹方向的轨迹长度
  double neighbor_expand_length_ = 1.0;  // 窄道轨迹延长距离

  std::shared_ptr<AngleTopathSupervisor> ptr_angle_supor_ = nullptr;
  std::shared_ptr<DistanceToPathSupervisor> ptr_distance_supor_ = nullptr;
  std::shared_ptr<FinalGoalSupervisor> ptr_goal_supor_ = nullptr;
  std::shared_ptr<ObstacleAvoiderSupervisor> ptr_obstacle_supor_ = nullptr;
  std::shared_ptr<SpeedLevelSupervisor> ptr_speed_supor_ = nullptr;
  std::shared_ptr<OverTimeSupervisor> ptr_time_supor_ = nullptr;
  std::shared_ptr<MarkerMapSupervisor> ptr_marker_supor_ = nullptr;
  std::shared_ptr<PassPitSupervisor> ptr_pit_supor_ = nullptr;

  std::shared_ptr<State> ptr_current_state_ = nullptr;
  std::shared_ptr<PathManager> ptr_path_manager_ = nullptr;
  std::shared_ptr<BasePathRecovery> ptr_path_recovery_ = nullptr;
  std::shared_ptr<LocalController> ptr_local_controller_ = nullptr;
  std::shared_ptr<NavigationMediator> ptr_navigation_mediator_ =
      nullptr;  ///< 获取参数用
  std::shared_ptr<Costmap2d> ptr_costmap_2d_ = nullptr;

  bool forward_ = true;
  // bool normal_path_ = true;
  MISSIONTYPE mission_type_ = MISSIONTYPE::TRACKING;
  bool need_final_rotate_ = false;
};

}  // namespace CVTE_BABOT

#endif  // __PATH_FOLLOWER_HPP
