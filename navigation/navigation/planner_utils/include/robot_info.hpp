/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file robot_info.hpp
 *
 *@brief 保存机器人的状态和信息（位置、速度和状态等）
 *       其它地方要获取机器人基本信息，都是从这个类接口获取和设置
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 06
 ************************************************************************/

#ifndef __ROBOT_INFO_HPP
#define __ROBOT_INFO_HPP

#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <map>

#include "planner_utils.hpp"
#include "pose2d/pose2d.hpp"

namespace CVTE_BABOT {

/*********************************************
 使用单例模式，方便任意模块中获取机器人的实时基本信息
*********************************************/
enum ROBOTTYPE { KAVA_SECURITY, KAVA_CLEAN_C5, KAVA_CLEAN_C3 };
enum ROBOTMOTIONSTATE { FORWARD, TURN_LEFT, TURN_RIGHT, RECOVER, PAUSE, STOP };
enum MISSIONTYPE { EDGE = 0, TRACKING = 1, SLOPE = 2, ELVATOR = 3 };
enum ActionType {
  IDEL,
  NAVIGAOL,
  CLEAN,
  SUPPLEMENTCLEAN,
  SECURITY,
};
enum AreaType {
  clean_area,
  slope_area,
  elevator_area,
  carpet_area,
  narrow_area,
  black_area,
  prohibited_area,
  none,
};

class RobotInfo {
 public:
  /**
   *getPtrInstance
   *@brief
   *  获取当前的单例指针
   **/
  static std::shared_ptr<RobotInfo> getPtrInstance();

  /**
   *setCurrentPose
   *@brief
   *  更新定位信息
   *
   *@param[in] current_pose - 机器人在世界坐标系下的定位信息
   **/
  void setCurrentPose(const Pose2d &current_pose);

  /**
   * @brief 设置任务类型
   *
   * @param[] type -
   */

  inline void setMissionType(const MISSIONTYPE &type) { mission_type_ = type; }
  /**
   * @brief 获得当前任务类型
   *
   * @return MISSIONTYPE -
   */
  inline MISSIONTYPE getMissionType() const { return mission_type_; }

  inline void setActionType(const ActionType &type) { action_type_ = type; }

  inline ActionType getActionType() const { return action_type_; }

  inline void setAbsReach(bool abs_reach) { abs_reach_ = abs_reach; }

  inline bool getAbsReach() { return abs_reach_; }

  inline void setFinalRotate(bool final_rotate) {
    final_rotate_ = final_rotate;
  }

  inline bool getFinalRotate() { return final_rotate_; }

  inline void setForward(bool forward) { forward_ = forward; }

  inline bool getForward() { return forward_; }

  /**
   *setCurrentVelocity
   *@brief
   *  更新速度信息，外部从/odom话题获取，并设置进来
   *
   *@param[in] current_velocity - 机器人当前速度信息
   **/
  void setCurrentVelocity(const Velocity &current_velocity);

  /**
   *setRobotMotionState
   *@brief
   *  设置机器人运动状态
   *
   *@param[in] 设置机器人运动状态
   **/

  void setRobotMotionState(const ROBOTMOTIONSTATE &motion_state);

  /**
   *inputScan
   *@brief
   *  输入激光scan
   *
   *@param[in] scan - 机器人收到的scan
   **/

  void inputScan(const std::vector<Pose2d> &scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_ = scan;
  }

  /**
   *inputSPDValue
   *@brief
   *  输入psd数据
   *
   *@param[in] psd - 机器人收到的psd数据
   **/

  void inputSPDValue(const std::vector<float> &psd) {
    std::lock_guard<std::mutex> lock(psd_mutex_);

    if (psd_.empty()) {
      psd_ = std::vector<float>(4, -0.6);
    }
    // 低通
    float weight = 0.1;
    // 装在右侧 采到的数据为负
    psd_[2] = ((1.0 - weight) * psd_[2]) + ((weight) * -psd[2]);
    psd_[3] = ((1.0 - weight) * psd_[3]) + ((weight) * -psd[3]);
  }

  /**
   *inputSPDValue
   *@brief
   *  输入psd数据
   *
   *@param[in] psd - 机器人收到的psd数据
   **/

  void inputSPDValue(int index, double value) {
    std::lock_guard<std::mutex> lock(psd_mutex_);
    if (psd_.empty()) {
      psd_ = std::vector<float>(4, -0.6);
    }
    // 低通
    float weight = 0.6;
    // 装在右侧 采到的数据为负
    if (index == 0) {
      psd_[2] = ((1.0 - weight) * psd_[2]) + ((weight) * -value);
    } else {
      psd_[3] = ((1.0 - weight) * psd_[3]) + ((weight) * -value);
    }
    // if (index == 0) {
    //   psd_[2] = ((1.0 - weight) * psd_[2]) + ((weight) *value);
    // } else {
    //   psd_[3] = ((1.0 - weight) * psd_[3]) + ((weight) *value);
    // }
  }

  /**
   *inputCameraCloud
   *@brief
   *  输入相机点云
   *
   *@param[in] point_cloud - 点云 已转换为pose2D
   **/

  void inputCameraCloud(const std::vector<Pose2d> &point_cloud) {
    std::lock_guard<std::mutex> lock(point_cloud_mutex_);
    point_cloud_ = point_cloud;
  }

  /**
   * @brief 输入避障红外信息
   *
   * @param[in] infrared_range
   */
  void inputInfraredRange(std::string frame_id, const double &infrared_range) {
    //是否加锁，加锁的意义
    // std::lock_guard<std::mutex> lock(infrared_obstacle_mutex_);
    infrared_obstacle_range_[frame_id] = infrared_range;

  }

  /**
   *getCurrentPose
   *@brief
   *  获取机器人位置信息
   *
   *@return 机器人在世界坐标系下的定位信息
   **/
  Pose2d getCurrentPoseWithOffset();

  /**
   *getCurrentPose
   *@brief
   *  获取机器人位置信息
   *
   *@return 机器人在世界坐标系下的定位信息
   **/
  Pose2d getCurrentPose();

  /**
   *getRobotMotionState
   *@brief
   *  获取机器人运动状态
   *
   *@return 机器人运动状态
   **/
  std::string getRobotMotionState();

  /**
   *getCurrentVel
   *@brief
   *  获取机器人速度信息
   *
   *@return 机器人当前速度信息
   **/
  Velocity getCurrentVel();

  /**
   *getScan
   *@brief
   *  获取机器人接收到的scan信息
   *
   *@return 机器人接收到的can
   **/
  std::vector<Pose2d> getScan() {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return scan_;
  }

  /**
   *getPSDValue
   *@brief
   *  获取机器人接收到的spd数据
   *
   *@return 机器人接收到的can
   **/
  std::vector<float> getPSDValue() {
    std::lock_guard<std::mutex> lock(psd_mutex_);
    return psd_;
  }

  /**
   *getCameraCloud
   *@brief
   *  获取机器人接收到的scan信息
   *
   *@return 机器人接收到的can
   **/
  std::vector<Pose2d> getCameraCloud() {
    std::lock_guard<std::mutex> lock(point_cloud_mutex_);
    return point_cloud_;
  }

  /**
   * @brief getInfraredObstacleRange
   *
   * @param type
   */
  std::map<std::string, double> getInfraredObstacleRange() {
    //加锁？
    return infrared_obstacle_range_;
  }

  void setRobotType(ROBOTTYPE type);
  void setOffsetPose(const Pose2d &offset_pose);
  ROBOTTYPE getRobotType();

 private:
  RobotInfo();
  RobotInfo(const RobotInfo &obj) = delete;
  RobotInfo &operator=(const RobotInfo &obj) = delete;

  static std::shared_ptr<RobotInfo> rb_info_ptr_;

  Pose2d current_pose_;  ///< 机器人世界坐标系下的位置
  Pose2d offset_pose_;
  Velocity current_velocity_;  ///< 机器人当前速度信息，从odom话题获取
  ROBOTMOTIONSTATE motion_state_;

  std::map<ROBOTMOTIONSTATE, std::string> motion_state2string_;

  std::vector<Pose2d> scan_;
  std::vector<Pose2d> point_cloud_;
  std::map<std::string, double> infrared_obstacle_range_;
  std::vector<float> psd_;

  std::mutex scan_mutex_;
  std::mutex psd_mutex_;
  std::mutex point_cloud_mutex_;

  std::mutex infrared_obstacle_mutex_;

  std::mutex current_pose_mutex_;      ///< 位置锁
  std::mutex current_velocity_mutex_;  ///< 速度锁

  std::mutex robot_type_mutex_;  ///< 类型锁

  std::mutex robot_motion_state_mutex_;
  ROBOTTYPE robot_type_ = KAVA_SECURITY;
  MISSIONTYPE mission_type_ = MISSIONTYPE::TRACKING;
  ActionType action_type_ = ActionType::NAVIGAOL;
  bool abs_reach_ = false;
  bool final_rotate_ = false;
  bool forward_ = true;
};

}  // namespace CVTE_BABOT

#endif  // end of __ROBOT_INFO_HPP