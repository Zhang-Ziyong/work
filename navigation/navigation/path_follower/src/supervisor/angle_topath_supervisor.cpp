#include "supervisor/angle_topath_supervisor.hpp"
#include "path_follower.hpp"

#include <float.h>
#include <glog/logging.h>

namespace CVTE_BABOT {
AngleTopathSupervisor ::AngleTopathSupervisor(double max_angle_to_path)
    : max_angel_(max_angle_to_path) {}

void AngleTopathSupervisor::supervise(const SupervisorState &state,
                                      SupervisorResult &out) {
  double angle = calculateAngleToCurrentPathSegment(state, out);
  LOG(INFO) << "AngleToPathSupervisor distance: " << angle;

  if (fabs(angle) > max_angel_ && angle != DBL_MAX &&
      (RobotInfo::getPtrInstance()->getRobotType() ==
           ROBOTTYPE::KAVA_CLEAN_C3 ||
       RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::EDGE)) {
    LOG(INFO) << "angle too fat ";
    out.can_continue = false;
    out.new_local_goal = false;
    out.target_point.position.setYaw(traget_yaw_);
    out.status = ANGLETOPATH;
    out.supervisor_name = "AnglePathFaraway";
  }
}

double AngleTopathSupervisor::calculateAngleToCurrentPathSegment(
    const SupervisorState &state, SupervisorResult &out) {
  std::shared_ptr<SubPath> sub_path = state.sub_path_;
  if (sub_path == nullptr || sub_path->wps.empty()) {
    return 0.0;
  }

  double angle_diff = 0;
  Pose2d inv_current_pose =
      RobotInfo::getPtrInstance()->getCurrentPose().inverse();
  size_t current_index = (sub_path->type == PathType::REFER)
                             ? state.pf_ptr_->getPmPtr()->getReferIndex()
                             : state.pf_ptr_->getPmPtr()->getLocalIndex();

  traget_yaw_ = sub_path->wps[current_index].getYaw();
  angle_diff = (inv_current_pose * sub_path->wps[current_index]).getYaw();

  return angle_diff;
  /*
  int index = 0;
  double dist = 0.0;
  double sum_dist = 0.0;
  double angle_diff = 0.0;
  double min_angle_diff = DBL_MAX;
  double min_dist = DBL_MAX;
  Pose2d inv_current_pose =
      RobotInfo::getPtrInstance()->getCurrentPose().inverse();

  //
  如果当前检查的是任务路径，则只检查当前索引到往后5米的范围，如果是局部路径则检查整段的最小值
  size_t current_refer_index = state.pf_ptr_->getPmPtr()->getReferIndex();
  size_t index_start =
      (sub_path->type == PathType::REFER) ? current_refer_index : 0;
  size_t index_end =
      (sub_path->type == PathType::REFER)
          ? state.pf_ptr_->getPmPtr()->calcReferIndexWithDistance(3.0)
          : sub_path->wps.size();

  // 找到最近的点
  for (size_t i = index_start; i < index_end; ++i) {
    dist = state.robot_pose_.distanceTo(sub_path->wps[i]);
    if (dist < min_dist) {
      min_dist = dist;
      index = i;
    }
  }

  angle_diff = (inv_current_pose * sub_path->wps[index]).getYaw();
  if (fabs(angle_diff) < max_angel_) {
    return angle_diff;
  }

  // 找最小角度差
  // TODO: 角度获取错误
  size_t j;
  for (j = index; j < index_end; ++j) {
    // 获取前方0.5的点
    sum_dist += sub_path->wps[j].distanceTo(sub_path->wps[j + 1]);
    if (sum_dist > 1.0) {
      break;
    }
    angle_diff = (inv_current_pose * sub_path->wps[j]).getYaw();
    if (min_angle_diff > fabs(angle_diff)) {
      min_angle_diff = fabs(angle_diff);
      traget_yaw_ = sub_path->wps[j].getYaw();
    }
  }

  return min_angle_diff;
  */
}

}  // namespace CVTE_BABOT
