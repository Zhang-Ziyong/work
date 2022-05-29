/*
 * @Author: linyanlong
 * @Date: 2021-08-19 11:17:12
 * @LastEditTime: 2021-10-18 10:22:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/planner_decision/src/planner_decison.cpp
 */
#include "planner_decision/planner_decision.hpp"
#include <algorithm>
namespace CVTE_BABOT {
std::shared_ptr<PlannerDecision> PlannerDecision::ptr_planner_decision_ =
    nullptr;

std::shared_ptr<PlannerDecision> PlannerDecision::getInstance() {
  if (ptr_planner_decision_ == nullptr) {
    ptr_planner_decision_.reset(new PlannerDecision());
    return ptr_planner_decision_;
  } else {
    return ptr_planner_decision_;
  }
}
bool PlannerDecision::getTargetPoint(const Pose2d &robot_pose,
                                     PathFollower *pf_ptr,
                                     const FollowerStateRes &state,
                                     PoseMsg &target_pose_msg) {
  LOG(INFO) << "get target point";
  robot_pose_ = robot_pose;
  if (pf_ptr == nullptr) {
    LOG(INFO) << "pf_ptr is null";
    return false;
  }
  ptr_costmap_ = pf_ptr->getPmPtr()->getCostmap();
  ptr_refer_path_ = pf_ptr->getPmPtr()->getReferPath();
  if (ptr_refer_path_ == nullptr || ptr_costmap_ == nullptr) {
    LOG(ERROR) << "ptr_refer_path_ is null or ptr_costmap is null";
    return false;
  }
  if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
    target_pose_msg.position = ptr_refer_path_->wps.back();
    target_pose_msg.index = ptr_refer_path_->wps.size() - 1;
    return true;
  }
  if (state.states == PathFollowerStates::FOLLOW_REFERENCE ||
      (state.states == PathFollowerStates::PAUSE &&
       state.msg == "ObstacleAvoider")) {
    return getAvoiderTargetPoint(pf_ptr, target_pose_msg);
  } else if (state.states == PathFollowerStates::FOLLOW_LOCAL) {
    return getBetterTargetPoint(pf_ptr, target_pose_msg);
  } else if ((state.states == PathFollowerStates::PASS_PIT)) {
    return getPitTargetPoint(pf_ptr, target_pose_msg);
  } else {
    return getRecoveryTargetPoint(pf_ptr, target_pose_msg);
  }
}
void PlannerDecision::inputReferPath(std::shared_ptr<SubPath> ptr_refer_path) {
  ptr_refer_path_ = ptr_refer_path;
}
void PlannerDecision::setConfig(const PlannerDecisionConfig &config) {
  config_ = config;
  LOG(INFO) << "planner_decision_config: ";
  LOG(INFO) << "bussiness_dist: " << config_.bussiness_dist;
  LOG(INFO) << "bussiness_range: " << config_.bussiness_range;
  LOG(INFO) << "dynamic_dist: " << config_.dynamic_dist;
  LOG(INFO) << "map_range: " << config_.map_range;
  LOG(INFO) << "step: " << config_.step;
  LOG(INFO) << "k_bussiness: " << config_.k_bussiness;
  LOG(INFO) << "k_safe: " << config_.k_safe;
  LOG(INFO) << "k_dynamic: " << config_.k_dynamic;
  LOG(INFO) << "k_mission: " << config_.k_mission;
  LOG(INFO) << "check map ranger: " << config_.check_map_range;
  LOG(INFO) << "foot_print1: " << config_.foot_print[0](0) << " "
            << config_.foot_print[0](1);
  LOG(INFO) << "foot_print2: " << config_.foot_print[1](0) << " "
            << config_.foot_print[1](1);
  LOG(INFO) << "max_cost: " << config_.max_cost;
  LOG(INFO) << "min_cost: " << config_.min_cost;
}
int PlannerDecision::checkPointCostValue(
    const Pose2d &check_point, std::shared_ptr<Costmap2d> ptr_costmap_2d) {
  return ptr_costmap_2d->getCostWithMapPose(check_point.getX(),
                                            check_point.getY());
}

bool PlannerDecision::getAvoiderTargetPoint(PathFollower *pf_ptr,
                                            PoseMsg &target_pose_msg) {
  LOG(INFO) << "get obstacle avoider target";
  std::vector<PoseWithFactor> vec_candidate_target;
  bool crash = false;
  size_t begin_index = pf_ptr->getPmPtr()->getReferIndex();
  size_t last_index = begin_index;
  size_t crash_index = begin_index;
  double x_size = ptr_costmap_->getSizeInCellsX();
  double y_size = ptr_costmap_->getSizeInCellsY();
  std::shared_ptr<DijkstraExpansion> dijkstar_planner_ptr_ =
      std::make_shared<DijkstraExpansion>(x_size, y_size, false);
  dijkstar_planner_ptr_->outlineMap(ptr_costmap_->getCharMap(), x_size, y_size,
                                    253);
  WorldmapPoint start_wm_point = {robot_pose_.getX(), robot_pose_.getY()};
  CostmapPoint start_cp_point;
  if (!ptr_costmap_->worldToMap(start_wm_point, start_cp_point)) {
    return false;
  }
  //获取当前机器人位置的代价值,以此设定势场的致命值
  int cur_cost = static_cast<int>(
      ptr_costmap_->getCost(start_cp_point.ui_x, start_cp_point.ui_y));
  LOG(INFO) << "planner_decision current position cost: " << cur_cost;
  int lethal_cost =
      ((cur_cost > config_.min_cost) && (cur_cost < config_.max_cost))
          ? cur_cost
          : config_.min_cost;
  LOG(INFO) << "planner_decision lethal cost: " << lethal_cost;
  dijkstar_planner_ptr_->calculateAllPotentials(
      ptr_costmap_->getCharMap(), start_cp_point.ui_x, start_cp_point.ui_y,
      x_size * y_size * 2, lethal_cost);
  // LOG(INFO) << "ptr_refer_path_->wps.size(): " <<
  // ptr_refer_path_->wps.size();
  for (size_t index = begin_index; index < ptr_refer_path_->wps.size();
       index++) {
    // LOG(INFO) << "check index: " << index;
    if (config_.check_map_range && ptr_refer_path_->wps[index].distanceTo(
                                       robot_pose_) > config_.map_range) {
      if (vec_candidate_target.empty()) {
        // PoseWithFactor candidate_pose;
        // candidate_pose.pose.position = ptr_refer_path_->wps[index - 1];
        // candidate_pose.pose.index = index - 1;
        // candidate_pose.pose.velocity = ptr_refer_path_->wpis[index - 1];
        // candidate_pose.factor = 1.0;
        // if (refineClearPoint(candidate_pose.pose)) {
        //   LOG(INFO) << "refine clear point succeed";
        //   vec_candidate_target.push_back(candidate_pose);
        // } else {
        //   LOG(INFO) << "refine clear point failed";
        // }
      }
      break;
    }
    if ((ptr_refer_path_->wpis[index].path_length -
                 ptr_refer_path_->wpis[begin_index].path_length >
             config_.map_range &&
         !vec_candidate_target.empty())) {
      //目标点超出地图范围后在此处处理
      // LOG(INFO) << "out of map range";
      // LOG(INFO) << "robot_pose: " << robot_pose_.getX() << " "
      //           << robot_pose_.getY();
      // LOG(INFO) << "refer_pose: " << ptr_refer_path_->wps[index].getX() << "
      // "
      //           << ptr_refer_path_->wps[index].getY();
      break;
    } else {
      if (!crash) {
        if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) >
            config_.dangerous_value) {
          LOG(INFO) << "crash index: " << index
                    << " pose: " << ptr_refer_path_->wps[index].getX() << " "
                    << ptr_refer_path_->wps[index].getY();
          crash = true;
          last_index = index;
          crash_index = index;
        }
      } else {
        if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) >
            config_.dangerous_value) {
          crash_index = index;
        }
        WorldmapPoint wm_point = {ptr_refer_path_->wps[index].getX(),
                                  ptr_refer_path_->wps[index].getY()};
        CostmapPoint cp_point;
        if (!ptr_costmap_->worldToMap(wm_point, cp_point)) {
          continue;
        }
        double djCost = dijkstar_planner_ptr_->getDistancePotential(
            cp_point.ui_x, cp_point.ui_y);
        if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) <
                config_.dangerous_value &&
            ptr_refer_path_->wpis[index].path_length -
                    ptr_refer_path_->wpis[last_index].path_length >
                config_.step &&
            djCost < 1.0e9) {
          last_index = index;
          double safe_factor = getSafeFactor(ptr_refer_path_->wps[index]);
          double bussiness_factor = getBussinessFactor(
              ptr_refer_path_->wpis[index].path_length -
              ptr_refer_path_->wpis[crash_index].path_length);
          double dynamic_factor = getDynamicFactor(ptr_refer_path_->wps[index]);
          double mission_factor =
              getMissionFactor(ptr_refer_path_->wpis[index].path_length -
                               ptr_refer_path_->wpis[begin_index].path_length);
          double factor = config_.k_safe * safe_factor +
                          config_.k_bussiness * bussiness_factor +
                          config_.k_dynamic * dynamic_factor +
                          config_.k_mission * mission_factor;
          LOG(INFO) << "index: " << index << " safe_factor: " << safe_factor
                    << " bussiness_factor: " << bussiness_factor
                    << " dynamic_factor: " << dynamic_factor;
          PoseWithFactor candidate_pose;
          candidate_pose.pose.position = ptr_refer_path_->wps[index];
          candidate_pose.pose.index = index;
          candidate_pose.pose.velocity = ptr_refer_path_->wpis[index];
          candidate_pose.factor = factor;
          vec_candidate_target.push_back(candidate_pose);
        }
      }
    }
  }
  if (vec_candidate_target.empty()) {
    LOG(ERROR) << "get obstacle avoider target failed";
    return false;
  } else {
    LOG(INFO) << "get obstacle avoider target succeed";
    LOG(INFO) << "candidate points size: " << vec_candidate_target.size();
    std::sort(
        vec_candidate_target.begin(), vec_candidate_target.end(),
        [](const PoseWithFactor &left, const PoseWithFactor &right) -> bool {
          return left.factor > right.factor;
        });
  }
  target_pose_msg = vec_candidate_target[0].pose;
  return true;
}
bool PlannerDecision::getBetterTargetPoint(PathFollower *pf_ptr,
                                           PoseMsg &target_pose_msg) {
  LOG(INFO) << "get better target point";
  std::vector<PoseWithFactor> vec_candidate_target;
  bool crash = false;
  size_t begin_index = pf_ptr->getPmPtr()->getReferIndex();
  size_t target_index = pf_ptr->getPmPtr()->getTargetIndex();
  size_t last_index = begin_index;
  size_t crash_index = begin_index;
  double x_size = ptr_costmap_->getSizeInCellsX();
  double y_size = ptr_costmap_->getSizeInCellsY();
  std::shared_ptr<DijkstraExpansion> dijkstar_planner_ptr_ =
      std::make_shared<DijkstraExpansion>(x_size, y_size, false);
  dijkstar_planner_ptr_->outlineMap(ptr_costmap_->getCharMap(), x_size, y_size,
                                    253);
  WorldmapPoint start_wm_point = {robot_pose_.getX(), robot_pose_.getY()};
  CostmapPoint start_cp_point;
  if (!ptr_costmap_->worldToMap(start_wm_point, start_cp_point)) {
    return false;
  }
  //获取当前机器人位置的代价值,以此设定势场的致命值
  int cur_cost = static_cast<int>(
      ptr_costmap_->getCost(start_cp_point.ui_x, start_cp_point.ui_y));
  LOG(INFO) << "planner_decision current position cost: " << cur_cost;
  int lethal_cost = (cur_cost > config_.min_cost && cur_cost < config_.max_cost)
                        ? cur_cost
                        : config_.min_cost;
  LOG(INFO) << "planner_decision lethal cost: " << lethal_cost;
  dijkstar_planner_ptr_->calculateAllPotentials(
      ptr_costmap_->getCharMap(), start_cp_point.ui_x, start_cp_point.ui_y,
      x_size * y_size * 2, lethal_cost);
  for (size_t index = begin_index; index < ptr_refer_path_->wps.size();
       index++) {
    if (config_.check_map_range && ptr_refer_path_->wps[index].distanceTo(
                                       robot_pose_) > config_.map_range) {
      if (vec_candidate_target.empty()) {
        //目标点超出地图范围后在此处处理
        // PoseWithFactor candidate_pose;
        // candidate_pose.pose.position = ptr_refer_path_->wps[index - 1];
        // candidate_pose.pose.index = index - 1;
        // candidate_pose.pose.velocity = ptr_refer_path_->wpis[index - 1];
        // candidate_pose.factor = 1.0;
        // if (refineClearPoint(candidate_pose.pose)) {
        //   LOG(INFO) << "refine clear point succeed";
        //   vec_candidate_target.push_back(candidate_pose);
        // } else {
        //   LOG(INFO) << "refine clear point failed";
        // }
      }
      break;
    }
    if ((ptr_refer_path_->wpis[index].path_length -
                 ptr_refer_path_->wpis[begin_index].path_length >
             config_.map_range &&
         !vec_candidate_target.empty())) {
      break;
    } else if (index == target_index && !crash) {
      LOG(INFO) << "no need to update target point";
      break;
    } else {
      if (!crash) {
        if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) >
            config_.dangerous_value) {
          LOG(INFO) << "crash index: " << index
                    << " pose: " << ptr_refer_path_->wps[index].getX() << " "
                    << ptr_refer_path_->wps[index].getY();
          crash = true;
          last_index = index;
          crash_index = index;
        }
      } else {
        WorldmapPoint wm_point = {ptr_refer_path_->wps[index].getX(),
                                  ptr_refer_path_->wps[index].getY()};
        CostmapPoint cp_point;
        if (!ptr_costmap_->worldToMap(wm_point, cp_point)) {
          continue;
        }
        double djCost = dijkstar_planner_ptr_->getDistancePotential(
            cp_point.ui_x, cp_point.ui_y);
        if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) >
            config_.dangerous_value) {
          crash_index = index;
        }
        if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) <
                config_.dangerous_value &&
            ptr_refer_path_->wpis[index].path_length -
                    ptr_refer_path_->wpis[last_index].path_length >
                config_.step &&
            djCost < 1.0e9) {
          last_index = index;
          double safe_factor = getSafeFactor(ptr_refer_path_->wps[index]);
          double bussiness_factor = getBussinessFactor(
              ptr_refer_path_->wpis[index].path_length -
              ptr_refer_path_->wpis[crash_index].path_length);
          double dynamic_factor = getDynamicFactor(ptr_refer_path_->wps[index]);
          double mission_factor =
              getMissionFactor(ptr_refer_path_->wpis[index].path_length -
                               ptr_refer_path_->wpis[begin_index].path_length);
          double factor = config_.k_safe * safe_factor +
                          config_.k_bussiness * bussiness_factor +
                          0.0 * dynamic_factor +
                          config_.k_mission * mission_factor;
          LOG(INFO) << "candidate index: " << index
                    << " safe_factor: " << safe_factor
                    << " bussiness_factor: " << bussiness_factor
                    << " dynamic_factor: " << dynamic_factor;
          PoseWithFactor candidate_pose;
          candidate_pose.pose.position = ptr_refer_path_->wps[index];
          candidate_pose.pose.index = index;
          candidate_pose.pose.velocity = ptr_refer_path_->wpis[index];
          candidate_pose.factor = factor;
          vec_candidate_target.push_back(candidate_pose);
        }
      }
    }
  }
  if (vec_candidate_target.empty()) {
    LOG(INFO) << "get better target point failed";
    return false;
  } else {
    LOG(INFO) << "get better target point succeed";
    std::sort(
        vec_candidate_target.begin(), vec_candidate_target.end(),
        [](const PoseWithFactor &left, const PoseWithFactor &right) -> bool {
          return left.factor > right.factor;
        });
  }
  target_pose_msg = vec_candidate_target[0].pose;
  return true;
}
bool PlannerDecision::getRecoveryTargetPoint(PathFollower *pf_ptr,
                                             PoseMsg &target_pose_msg) {
  LOG(INFO) << "get recover target point";
  std::vector<PoseWithFactor> vec_candidate_target;
  bool crash = false;
  size_t begin_index = pf_ptr->getPmPtr()->getReferIndex();
  size_t crash_index = begin_index;
  size_t last_index = begin_index;
  LOG(INFO) << "begin index: " << begin_index;
  double x_size = ptr_costmap_->getSizeInCellsX();
  double y_size = ptr_costmap_->getSizeInCellsY();
  std::shared_ptr<DijkstraExpansion> dijkstar_planner_ptr_ =
      std::make_shared<DijkstraExpansion>(x_size, y_size, false);
  dijkstar_planner_ptr_->outlineMap(ptr_costmap_->getCharMap(), x_size, y_size,
                                    253);
  WorldmapPoint start_wm_point = {robot_pose_.getX(), robot_pose_.getY()};
  CostmapPoint start_cp_point;
  if (!ptr_costmap_->worldToMap(start_wm_point, start_cp_point)) {
    return false;
  }
  //获取当前机器人位置的代价值,以此设定势场的致命值
  int cur_cost = static_cast<int>(
      ptr_costmap_->getCost(start_cp_point.ui_x, start_cp_point.ui_y));
  LOG(INFO) << "planner_decision current position cost: " << cur_cost;
  int lethal_cost = (cur_cost > config_.min_cost && cur_cost < config_.max_cost)
                        ? cur_cost
                        : config_.min_cost;
  LOG(INFO) << "planner_decision lethal cost: " << lethal_cost;
  dijkstar_planner_ptr_->calculateAllPotentials(
      ptr_costmap_->getCharMap(), start_cp_point.ui_x, start_cp_point.ui_y,
      x_size * y_size * 2, lethal_cost);
  LOG(INFO) << "refer path size: " << ptr_refer_path_->wps.size();
  for (size_t index = begin_index; index < ptr_refer_path_->wps.size();
       index++) {
    if (config_.check_map_range && ptr_refer_path_->wps[index].distanceTo(
                                       robot_pose_) > config_.map_range) {
      LOG(INFO) << "candidate point out of map";
      if (vec_candidate_target.empty()) {
        //目标点超过地图范围后在此处处理
      }
      break;
    }
    if ((ptr_refer_path_->wpis[index].path_length -
                 ptr_refer_path_->wpis[begin_index].path_length >
             config_.map_range &&
         !vec_candidate_target.empty())) {
      LOG(INFO) << "candidate point is not empty and out of map range";
      break;
    } else {
      LOG(INFO) << "recovery find candidate";
      WorldmapPoint wm_point = {ptr_refer_path_->wps[index].getX(),
                                ptr_refer_path_->wps[index].getY()};
      CostmapPoint cp_point;
      if (!ptr_costmap_->worldToMap(wm_point, cp_point)) {
        continue;
      }
      double djCost = dijkstar_planner_ptr_->getDistancePotential(
          cp_point.ui_x, cp_point.ui_y);
      if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) >
          config_.dangerous_value) {
        crash_index = index;
        LOG(INFO) << "crash index: " << index
                  << " pose: " << ptr_refer_path_->wps[index].getX() << " "
                  << ptr_refer_path_->wps[index].getY();
        if (!crash) {
          crash = true;
        }
      }
      if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) <
              config_.dangerous_value &&
          (ptr_refer_path_->wpis[index].path_length -
                   ptr_refer_path_->wpis[last_index].path_length >
               config_.step ||
           index == ptr_refer_path_->wps.size() - 1) &&
          djCost < 1.0e9) {
        last_index = index;
        double safe_factor = getSafeFactor(ptr_refer_path_->wps[index]);
        double bussiness_factor =
            getBussinessFactor(ptr_refer_path_->wpis[index].path_length -
                               ptr_refer_path_->wpis[crash_index].path_length);
        double dynamic_factor =
            2 * getDynamicFactor(ptr_refer_path_->wps[index]);
        double mission_factor =
            0.5 *
            getMissionFactor(ptr_refer_path_->wpis[index].path_length -
                             ptr_refer_path_->wpis[begin_index].path_length);
        double factor = config_.k_safe * safe_factor +
                        config_.k_bussiness * bussiness_factor +
                        config_.k_dynamic * dynamic_factor +
                        config_.k_mission * mission_factor;
        LOG(INFO) << "candidate index: " << index
                  << " safe_factor: " << safe_factor
                  << " bussiness_factor: " << bussiness_factor
                  << " dynamic_factor: " << dynamic_factor;
        PoseWithFactor candidate_pose;
        candidate_pose.pose.position = ptr_refer_path_->wps[index];
        candidate_pose.pose.index = index;
        candidate_pose.pose.velocity = ptr_refer_path_->wpis[index];
        candidate_pose.factor = factor;
        vec_candidate_target.push_back(candidate_pose);
      }
    }
  }
  if (vec_candidate_target.empty()) {
    LOG(INFO) << "get recovery target point failed";
    return false;
  } else {
    LOG(INFO) << "get recovery target point succeed";
    std::sort(
        vec_candidate_target.begin(), vec_candidate_target.end(),
        [](const PoseWithFactor &left, const PoseWithFactor &right) -> bool {
          return left.factor > right.factor;
        });
  }
  target_pose_msg = vec_candidate_target[0].pose;
  return true;
}

bool PlannerDecision::getPitTargetPoint(PathFollower *pf_ptr,
                                        PoseMsg &target_pose_msg) {
  LOG(INFO) << "get recover target point";
  std::vector<PoseWithFactor> vec_candidate_target;

  size_t start_index = pf_ptr->getPmPtr()->getReferPathSize() - 1;

  size_t begin_index = pf_ptr->getPmPtr()->getReferIndex();  //不是getReferIndex
  auto r_path = pf_ptr->getPmPtr()->getReferPath();
  for (size_t i = begin_index; i < pf_ptr->getPmPtr()->getReferPathSize();
       i += 5) {
    if (r_path->wpis[i].path_length - r_path->wpis[begin_index].path_length >
        4) {
      start_index = i;
      break;
    }
  }

  size_t last_index = start_index;
  LOG(INFO) << "start index: " << start_index;
  double x_size = ptr_costmap_->getSizeInCellsX();
  double y_size = ptr_costmap_->getSizeInCellsY();
  std::shared_ptr<DijkstraExpansion> dijkstar_planner_ptr_ =
      std::make_shared<DijkstraExpansion>(x_size, y_size, false);
  dijkstar_planner_ptr_->outlineMap(ptr_costmap_->getCharMap(), x_size, y_size,
                                    253);
  WorldmapPoint start_wm_point = {robot_pose_.getX(), robot_pose_.getY()};
  CostmapPoint start_cp_point;
  if (!ptr_costmap_->worldToMap(start_wm_point, start_cp_point)) {
    return false;
  }
  //获取当前机器人位置的代价值,以此设定势场的致命值
  int cur_cost = static_cast<int>(
      ptr_costmap_->getCost(start_cp_point.ui_x, start_cp_point.ui_y));
  LOG(INFO) << "planner_decision current position cost: " << cur_cost;
  int lethal_cost = (cur_cost > config_.min_cost && cur_cost < config_.max_cost)
                        ? cur_cost
                        : config_.min_cost;
  LOG(INFO) << "planner_decision lethal cost: " << lethal_cost;
  dijkstar_planner_ptr_->calculateAllPotentials(
      ptr_costmap_->getCharMap(), start_cp_point.ui_x, start_cp_point.ui_y,
      x_size * y_size * 2, lethal_cost);
  LOG(INFO) << "refer path size: " << ptr_refer_path_->wps.size();

  size_t crash_index = 0;
  for (size_t index = start_index; index < ptr_refer_path_->wps.size();
       index++) {
    if (config_.check_map_range && ptr_refer_path_->wps[index].distanceTo(
                                       robot_pose_) > config_.map_range) {
      LOG(INFO) << "candidate point out of map";
      if (vec_candidate_target.empty()) {}
      break;
    }
    if ((ptr_refer_path_->wpis[index].path_length -
                 ptr_refer_path_->wpis[start_index].path_length >
             config_.map_range &&
         !vec_candidate_target.empty())) {
      LOG(INFO) << "candidate point is not empty and out of map range";
      break;
    } else {
      WorldmapPoint wm_point = {ptr_refer_path_->wps[index].getX(),
                                ptr_refer_path_->wps[index].getY()};
      CostmapPoint cp_point;
      if (!ptr_costmap_->worldToMap(wm_point, cp_point)) {
        continue;
      }
      double djCost = dijkstar_planner_ptr_->getDistancePotential(
          cp_point.ui_x, cp_point.ui_y);
      if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) >
          config_.dangerous_value) {
        crash_index = index;
      }
      if (checkPointCostValue(ptr_refer_path_->wps[index], ptr_costmap_) <
              config_.dangerous_value &&
          (ptr_refer_path_->wpis[index].path_length -
                   ptr_refer_path_->wpis[last_index].path_length >
               config_.step ||
           index == ptr_refer_path_->wps.size() - 1) &&
          djCost < 1.0e9) {
        last_index = index;
        double safe_factor = getSafeFactor(ptr_refer_path_->wps[index]);
        double bussiness_factor =
            getBussinessFactor(ptr_refer_path_->wpis[index].path_length -
                               ptr_refer_path_->wpis[crash_index].path_length);
        double dynamic_factor =
            2 * getDynamicFactor(ptr_refer_path_->wps[index]);
        double mission_factor =
            0.5 *
            getMissionFactor(ptr_refer_path_->wpis[index].path_length -
                             ptr_refer_path_->wpis[start_index].path_length);
        double factor = config_.k_safe * safe_factor +
                        config_.k_bussiness * bussiness_factor +
                        config_.k_dynamic * dynamic_factor +
                        config_.k_mission * mission_factor;
        LOG(INFO) << "candidate index: " << index
                  << " safe_factor: " << safe_factor
                  << " bussiness_factor: " << bussiness_factor
                  << " dynamic_factor: " << dynamic_factor;
        PoseWithFactor candidate_pose;
        candidate_pose.pose.position = ptr_refer_path_->wps[index];
        candidate_pose.pose.index = index;
        candidate_pose.pose.velocity = ptr_refer_path_->wpis[index];
        candidate_pose.factor = factor;
        vec_candidate_target.push_back(candidate_pose);
      }
    }
  }
  if (vec_candidate_target.empty()) {
    LOG(INFO) << "get pit target point failed";
    return false;
  } else {
    LOG(INFO) << "get pit target point succeed";
    std::sort(
        vec_candidate_target.begin(), vec_candidate_target.end(),
        [](const PoseWithFactor &left, const PoseWithFactor &right) -> bool {
          return left.factor > right.factor;
        });
  }
  target_pose_msg = vec_candidate_target[0].pose;

  LOG(INFO) << "target = " << target_pose_msg.position.getX() << ","
            << target_pose_msg.position.getY()
            << ", yaw=" << target_pose_msg.position.getYaw();

  return true;
}

bool PlannerDecision::refineClearPoint(PoseMsg &target_pose) {
  Pose2d pose_start = target_pose.position;
  double max_dist = target_pose.position.distanceTo(robot_pose_);
  Pose2d candidate_pose = pose_start;
  for (double dist = 0; dist < max_dist; dist += config_.step) {
    // candidate_pose = pose_start + dist * (robot_pose_ - pose_start) /
    // max_dist;
    candidate_pose.setX(pose_start.getX() +
                        dist * (robot_pose_.getX() - pose_start.getX()) /
                            max_dist);
    candidate_pose.setY(pose_start.getY() +
                        dist * (robot_pose_.getY() - pose_start.getY()) /
                            max_dist);
    if (checkPointCostValue(candidate_pose, ptr_costmap_) <
        config_.dangerous_value) {
      break;
    }
  }
  if (checkPointCostValue(candidate_pose, ptr_costmap_) <
      config_.dangerous_value) {
    target_pose.position = candidate_pose;
    return true;
  } else {
    return false;
  }
}

double PlannerDecision::getSafeFactor(const Pose2d &target_pose) {
  Pose2d point1 = target_pose *
                  Pose2d(config_.foot_print[0](0), config_.foot_print[0](1), 0);
  Pose2d point2 = target_pose *
                  Pose2d(config_.foot_print[1](0), config_.foot_print[0](1), 0);
  Pose2d point3 = target_pose *
                  Pose2d(config_.foot_print[0](0), config_.foot_print[1](1), 0);
  Pose2d point4 = target_pose *
                  Pose2d(config_.foot_print[1](0), config_.foot_print[1](1), 0);
  double cost_sum = 4.0 * 254;
  double cost1 = checkPointCostValue(point1, ptr_costmap_);
  double cost2 = checkPointCostValue(point2, ptr_costmap_);
  double cost3 = checkPointCostValue(point3, ptr_costmap_);
  double cost4 = checkPointCostValue(point4, ptr_costmap_);
  // LOG(INFO) << "target point: " << target_pose.getX() << " "
  //           << target_pose.getY() << " " << target_pose.getYaw();
  // LOG(INFO) << "footprint1: " << point1.getX() << " " << point1.getY();
  // LOG(INFO) << "footprint2: " << point2.getX() << " " << point2.getY();
  // LOG(INFO) << "footprint3: " << point3.getX() << " " << point3.getY();
  // LOG(INFO) << "footprint4: " << point4.getX() << " " << point4.getY();
  if (cost1 >= 254 || cost2 >= 254 || cost3 >= 254 || cost4 >= 254) {
    return 0.0;
  } else {
    return 1.0 - (cost1 + cost2 + cost3 + cost4) / cost_sum;
  }
}
double PlannerDecision::getBussinessFactor(double target_dist) {
  double k_increase = 1.0 / (config_.bussiness_dist - config_.bussiness_range);
  double k_decrease = -k_increase;
  if (target_dist < config_.bussiness_dist - config_.bussiness_range &&
      target_dist >= 0) {
    return k_increase * target_dist;
  } else if (target_dist <= config_.bussiness_dist + config_.bussiness_range &&
             target_dist >= config_.bussiness_dist - config_.bussiness_range) {
    return 1.0;
  } else if (target_dist > config_.bussiness_dist + config_.bussiness_range &&
             target_dist <= 2 * config_.bussiness_dist) {
    return (k_decrease * (target_dist - config_.bussiness_dist -
                          config_.bussiness_range) +
            1.0);
  } else {
    return 0.0;
  }
}
double PlannerDecision::getDynamicFactor(const Pose2d &target_pose) {
  double dist = sqrt((target_pose.getX() - robot_pose_.getX()) *
                         (target_pose.getX() - robot_pose_.getX()) +
                     (target_pose.getY() - robot_pose_.getY()) *
                         (target_pose.getY() - robot_pose_.getY()));
  double k_increase = 1.0 / (config_.dynamic_dist);
  double k_decrease = -k_increase;
  if (dist < config_.dynamic_dist && dist >= 0) {
    return k_increase * dist;
  } else {
    return 1.0;
  }
  // double k_increase = 1.0 / (config_.dynamic_dist - config_.range);
  // double k_decrease = -k_increase;
  // if (dist < config_.dynamic_dist - config_.range && dist >= 0) {
  //   return k_increase * dist;
  // } else if (dist <= config_.dynamic_dist + config_.range &&
  //            dist >= config_.dynamic_dist - config_.range) {
  //   return 1.0;
  // } else if (dist > config_.dynamic_dist + config_.range &&
  //            dist <= 2 * config_.k_bussiness) {
  //   return (k_decrease * (dist - config_.dynamic_dist - config_.range) + 1);
  // } else {
  //   return 0.0;
  // }
}

double PlannerDecision::getMissionFactor(double length) {
  if (length > 3) {
    return 0.0;
  } else if (length < 1) {
    return 1.0;
  } else {
    return -0.5 * length + 1.5;
  }
}
}  // namespace CVTE_BABOT