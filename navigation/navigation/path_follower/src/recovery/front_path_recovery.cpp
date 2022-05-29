#include <glog/logging.h>
#include "robot_info.hpp"
#include "recovery/front_path_recovery.hpp"
#include "speed_decision_base.hpp"
#include <limits>

namespace CVTE_BABOT {
FrontPathRecovery::FrontPathRecovery() {}

void FrontPathRecovery::recoverClear() {}
void FrontPathRecovery::changeStateTo(RecoveryState traget_state) {}
/*===========================恢复相关==================================*/
bool FrontPathRecovery::checkIfNeedToRecover() {}

RecoveryState FrontPathRecovery::calculateRecoverVelocity(double &recover_v,
                                                          double &recover_w) {}
}  // namespace CVTE_BABOT

// #include <glog/logging.h>
// #include "robot_info.hpp"
// #include "recovery/front_path_recovery.hpp"
// #include "speed_decision_base.hpp"
// #include <limits>

// namespace CVTE_BABOT {
// FrontPathRecovery::FrontPathRecovery() {}

// /*===========================恢复相关==================================*/
// bool FrontPathRecovery::checkIfNeedToRecover() {
//   // if (ptr_obstacle_map_->isThereObsFront()) {
//   if (SpeedDecisionBase::getInstance()->isObstacleFront()) {
//     // 记录开始后退时的位置
//     if (!start_trans_recover_) {
//       recover_start_trans_pose_ =
//       RobotInfo::getPtrInstance()->getCurrentPose(); start_trans_recover_ =
//       true; state_ = RecoveryState::GO_RETREAT;
//     }
//     LOG(INFO) << "check out obstacle is front";
//     return true;
//   } else if (SpeedDecisionBase::getInstance()->isObstacleBack()) {
//     // 记录开始后退时的位置
//     if (!start_trans_recover_) {
//       recover_start_trans_pose_ =
//       RobotInfo::getPtrInstance()->getCurrentPose(); start_trans_recover_ =
//       true; state_ = RecoveryState::GO_ADVANCE;
//     }
//     LOG(INFO) << "check out obstacle is back";
//     return true;
//   } else if (SpeedDecisionBase::getInstance()->isObstacleLeft()) {
//     // 记录开始右转时的位置
//     if (!start_rot_recover_) {
//       recover_start_rot_pose_ =
//       RobotInfo::getPtrInstance()->getCurrentPose(); start_rot_recover_ =
//       true; state_ = RecoveryState::GO_RIGHT;
//     }
//     LOG(INFO) << "check out obstacle is left";
//     return true;
//   } else if (SpeedDecisionBase::getInstance()->isObstacleRight()) {
//     // 记录开始左转时的位置
//     if (!start_rot_recover_) {
//       recover_start_rot_pose_ =
//       RobotInfo::getPtrInstance()->getCurrentPose(); start_rot_recover_ =
//       true; state_ = RecoveryState::GO_LEFT;
//     }
//     LOG(INFO) << "check out obstacle is right";
//     return true;
//   } else {
//     LOG(INFO) << "no obs in stop area !";
//     start_trans_recover_ = false;
//     start_rot_recover_ = false;
//     return false;
//   }
// }

// RecoveryState FrontPathRecovery::calculateRecoverVelocity(double &recover_v,
//                                                           double &recover_w)
//                                                           {
//   recover_v = 0.00;
//   recover_w = 0.00;

//   // 判断后面是否有障碍,倒退距离过长不再倒退
//   Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
//   double recover_distance =
//   current_pose.distanceTo(recover_start_trans_pose_); double recover_angle =
//       fabs(current_pose.getYaw() - recover_start_rot_pose_.getYaw());

//   // 判断是否达到恢复距离和角度阈值
//   bool trans_recover_fail = false;
//   bool rot_recover_fail = false;
//   if (start_trans_recover_ && recover_distance > max_recover_distance_) {
//     LOG(INFO) << "recover_distance " << recover_distance << " > "
//               << max_recover_distance_;
//     trans_recover_fail = true;
//   }
//   if (start_rot_recover_ && recover_angle > max_recover_angle_) {
//     LOG(INFO) << "recover_angle is " << recover_angle << " > "
//               << max_recover_angle_;
//     rot_recover_fail = true;
//   }

//   // 恢复失败原因判断
//   if (trans_recover_fail && !start_rot_recover_) {
//     LOG(INFO) << "just try translate recover, but fail";
//     return RECOVERYERROR;
//   } else if (rot_recover_fail && !start_trans_recover_) {
//     LOG(INFO) << "just try rotate recover, but fail";
//     return RECOVERYERROR;
//   } else if (trans_recover_fail && rot_recover_fail) {
//     LOG(INFO) << "try rotate and trans recover all fail";
//     return RECOVERYERROR;
//   }

//   // 恢复线速度确定
//   if (start_trans_recover_ && !trans_recover_fail) {
//     if (RecoveryState::GO_RETREAT == state_ &&
//         !SpeedDecisionBase::getInstance()->isObstacleBack()) {
//       recover_v = -recover_v_;
//       return RECOVERYSUCCESS;
//     } else if (RecoveryState::GO_ADVANCE == state_ &&
//                !SpeedDecisionBase::getInstance()->isObstacleFront()) {
//       recover_v = recover_v_;
//       return RECOVERYSUCCESS;
//     }
//   }

//   // 恢复角速度确定
//   if (start_rot_recover_ && !rot_recover_fail) {
//     // 右旋转恢复时，只考虑实时的传感器障碍物信息
//     if (RecoveryState::GO_RIGHT == state_ &&
//         !SpeedDecisionBase::getInstance()->isSensorObstacleRight()) {
//       recover_w = recover_w_;
//       return RECOVERYSUCCESS;
//     }
//     // 左旋转恢复时，只考虑实时的传感器障碍物信息
//     else if (RecoveryState::GO_LEFT == state_ &&
//              !SpeedDecisionBase::getInstance()->isSensorObstacleLeft()) {
//       recover_w = -recover_w_;
//       return RECOVERYSUCCESS;
//     }
//   }

//   if (fabs(recover_w) < std::numeric_limits<double>::epsilon() &&
//       fabs(recover_v) < std::numeric_limits<double>::epsilon()) {
//     LOG(WARNING) << "state: " << static_cast<int>(state_) << ", is_obs_front:
//     "
//                  << SpeedDecisionBase::getInstance()->isObstacleFront()
//                  << ", is_obs_back: "
//                  << SpeedDecisionBase::getInstance()->isObstacleBack()
//                  << ", is_obs_left: "
//                  <<
//                  SpeedDecisionBase::getInstance()->isSensorObstacleLeft()
//                  << ", is_obs_right: "
//                  <<
//                  SpeedDecisionBase::getInstance()->isSensorObstacleRight()
//                  << ", recover fail !";
//     start_trans_recover_ = false;
//     start_rot_recover_ = false;
//     recover_v = 0.00;
//     recover_w = 0.00;
//     return RECOVERYERROR;
//   }
//   /*=================================================================*/
// }
// }  // namespace CVTE_BABOT
