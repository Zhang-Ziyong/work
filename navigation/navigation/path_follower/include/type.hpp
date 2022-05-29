/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file type.hpp
 *
 *@brief PathFollower中用到的类型定义, Costmap使用了前置声明,
 *使用到Costmap时需要包含costmap_2d.hpp头文件
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-06-08
 ************************************************************************/
#ifndef __TYPE_HPP
#define __TYPE_HPP
#include <memory>
#include <string>
#include <vector>

#include "path.hpp"

namespace CVTE_BABOT {

class Costmap2d;
class PathFollower;

enum class PathFollowerStates {
  FREE,
  FOLLOW_REFERENCE,
  FOLLOW_LOCAL,
  PASS_PIT,
  AVOID_CAR,
  ERROR,
  PAUSE,
  ROTATE,
  RECOVER,
  DONE,
  ELEVATOR_DONE,
};

enum SupervisorStatus {
  NOTHING,
  DISTANCETOPATH,
  ANGLETOPATH,
  OBSTACLEAVOIDER,
  OBSTACLESTOP,
  CARAVOIDER,
  UNREACHABLE,
  LASTPOINT,
  FINALGOAL,
  OVERTIME,
  MARKERMAP,
  PASSPIT
};

enum RecoveryState {
  GO_RIGHT,
  GO_LEFT,
  GO_ADVANCE,
  GO_RETREAT,
  GO_DEFAULT,
  RECOVERYSUCCESS,
  RECOVERYERROR,
  GO_ARC_RIGHT,
  GO_ARC_LEFT,
};

struct FollowerStateRes {
  PathFollowerStates states;
  std::string msg;
};

struct SupervisorState {
  typedef std::shared_ptr<SupervisorState> Ptr;

  SupervisorState(const Pose2d &robot_pose, std::shared_ptr<SubPath> sub_path,
                  const PathFollowerStates state,
                  std::shared_ptr<Costmap2d> costmap, PathFollower *pf_ptr)
      : robot_pose_(robot_pose),
        sub_path_(sub_path),
        state_(state),
        ptr_costmap_2d_(costmap),
        pf_ptr_(pf_ptr) {}
  Pose2d robot_pose_;
  std::shared_ptr<SubPath> sub_path_ = nullptr;
  PathFollowerStates state_ = PathFollowerStates::FREE;
  PathFollower *pf_ptr_;
  std::shared_ptr<Costmap2d> ptr_costmap_2d_ = nullptr;  ///< 代价地图
};

struct SupervisorResult {
  SupervisorResult() : can_continue(true), new_local_goal(false), status() {}
  bool can_continue;
  bool new_local_goal;
  PoseMsg start_point;
  PoseMsg target_point;
  SupervisorStatus status = NOTHING;
  std::string supervisor_name = "";
};

}  // namespace CVTE_BABOT
#endif
