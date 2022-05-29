/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file D_star.cpp
 *
 *@brief D星算法的实现
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#include "navigation_mediator.hpp"
#include "dijkstra/D_star.hpp"
#include "dijkstra/potential_calculator.hpp"

#include <algorithm>
#include <glog/logging.h>
#include <iostream>
#include <tgmath.h>

namespace CVTE_BABOT {

DijkstraExpansion::DijkstraExpansion(const unsigned int &nx,
                                     const unsigned int &ny, bool distance_map)
    : Expander(nx, ny) {
  int lethal_value = 130;
  NavigationMediator::getPtrInstance()->getParam(
      "path_follower.dangerous_value", lethal_value, 150);
  Expander::setLethalCost(static_cast<unsigned char>(lethal_value));
  factor_ = static_cast<float>(lethal_cost_ - neutral_cost_) /
            static_cast<float>(254);
  p_calc_ = std::make_shared<PotentialCalculator>(nx, ny);
  use_distance_map_ = distance_map;
  currentBuffer_ = boost::shared_array<int>(new int[PRIORITYBUFSIZE]);
  nextBuffer_ = boost::shared_array<int>(new int[PRIORITYBUFSIZE]);
  overBuffer_ = boost::shared_array<int>(new int[PRIORITYBUFSIZE]);
  setSize(nx, ny);
  if (currentBuffer_.get() == NULL || nextBuffer_.get() == NULL ||
      overBuffer_.get() == NULL) {
    LOG(FATAL) << "[DijkstraExpansion] Can't get enough memory.";
  }
  LOG(INFO) << "DijkstraExpansion Constructor Finish";
}

void DijkstraExpansion::setSize(const unsigned int &nx,
                                const unsigned int &ny) {
  Expander::setSize(nx, ny);
  if (pending_.get() == NULL) {
    pending_ = boost::shared_array<bool>(new bool[nx * ny]);
  }
  if (pending_.get() == NULL) {
    LOG(FATAL) << "[DijkstraExpansion] Can't get enough memory for flag.";
  }
}

inline void DijkstraExpansion::setNeutralCost(
    const unsigned char &neutral_cost) {
  neutral_cost_ = neutral_cost;
  priorityIncrement_ = 2 * neutral_cost_;
}

float DijkstraExpansion::getCost(
    const boost::shared_array<unsigned char> &costs, const unsigned int &n) {
  if (n > ns_) {
    return lethal_cost_;
  }
  float cost_value = costs[n];
  // 根据代价比例计算索引点n的代价值
  if (cost_value < lethal_cost_ - 1 || (unknown_ && cost_value == 255)) {
    cost_value = std::min(cost_value * factor_ + neutral_cost_,
                          static_cast<float>(lethal_cost_ - 1));
    return cost_value;
  }
  // 如果索引点n在障碍物里面，则返回最大值
  return 254;
}

void DijkstraExpansion::pushOver(boost::shared_array<unsigned char> costs,
                                 const unsigned int &n) {
  // 更新已经访问过的队列
  if (n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ &&
      overEnd_ < PRIORITYBUFSIZE) {
    overBuffer_[overEnd_++] = n;
    pending_[n] = true;
  }
}

void DijkstraExpansion::pushNext(boost::shared_array<unsigned char> costs,
                                 const unsigned int &n) {
  // 更新下次要访问节点的队列
  if (n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ &&
      nextEnd_ < PRIORITYBUFSIZE) {
    nextBuffer_[nextEnd_++] = n;
    pending_[n] = true;
  }
}

void DijkstraExpansion::pushCur(boost::shared_array<unsigned char> costs,
                                const unsigned int &n) {
  // 更新当前需要访问节点的队列
  if (n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ &&
      currentEnd_ < PRIORITYBUFSIZE) {
    currentBuffer_[currentEnd_++] = n;
    pending_[n] = true;
  }
}

void DijkstraExpansion::updateCell(
    const boost::shared_array<unsigned char> &costs, int n) {
  cells_visited_++;  // 已访问节点数量加一
  float cost_value = getCost(costs, n);
  if (cost_value >= lethal_cost_) {
    return;
  }
  // 计算n点栅格的势场值, 如果是用代价势场则用代价值计算，否则用距离值计算
  float value = use_distance_map_ ? map_resolution_ : cost_value;
  float pot = p_calc_->calculatePotential(potential_, value, n);

  if (pot < potential_[n]) {
    float le = INVSQRT2 * getCost(costs, n - 1);
    float re = INVSQRT2 * getCost(costs, n + 1);
    float ue = INVSQRT2 * getCost(costs, n - nx_);
    float de = INVSQRT2 * getCost(costs, n + nx_);
    potential_[n] = pot;
    // 将n点附近四个方向的节点推入优先级队列
    if (pot < threshold_) {
      if (potential_[n - 1] > pot + le)
        pushNext(costs, n - 1);
      if (potential_[n + 1] > pot + re)
        pushNext(costs, n + 1);
      if (potential_[n - nx_] > pot + ue)
        pushNext(costs, n - nx_);
      if (potential_[n + nx_] > pot + de)
        pushNext(costs, n + nx_);
    } else {
      if (potential_[n - 1] > pot + le)
        pushOver(costs, n - 1);
      if (potential_[n + 1] > pot + re)
        pushOver(costs, n + 1);
      if (potential_[n - nx_] > pot + ue)
        pushOver(costs, n - nx_);
      if (potential_[n + nx_] > pot + de)
        pushOver(costs, n + nx_);
    }
  }
}

bool DijkstraExpansion::outlineMap(boost::shared_array<unsigned char> costmap,
                                   const unsigned int &x_size,
                                   const unsigned int &y_size,
                                   const unsigned char &value) {
  if (costmap.get() == NULL) {
    LOG(ERROR) << "Receive an empty to outlineMap.";
    return false;
  }
  if (nx_ != x_size || ny_ != y_size) {
    LOG(ERROR) << "Map size is different.";
    return false;
  }
  auto costmap_ptr = costmap.get();
  // 第一行边界
  for (unsigned int i = 0; i < x_size; ++i) { *costmap_ptr++ = value; }
  // 最后一行边界
  costmap_ptr = costmap.get() + (y_size - 1) * x_size;
  for (unsigned int i = 0; i < x_size; ++i) { *costmap_ptr++ = value; }
  // 第一列边界
  costmap_ptr = costmap.get();
  for (unsigned int i = 0; i < y_size; ++i, costmap_ptr += x_size) {
    *costmap_ptr = value;
  }
  // 最右一列边界
  costmap_ptr = costmap.get() + x_size - 1;
  for (unsigned int i = 0; i < y_size; ++i, costmap_ptr += x_size) {
    *costmap_ptr = value;
  }
  return true;
}

float DijkstraExpansion::getDistancePotential(int pose_x, int pose_y) {
  unsigned int index = toIndex(pose_x, pose_y);
  return potential_[index];
}

bool DijkstraExpansion::calculatePotentials(
    const boost::shared_array<unsigned char> &costs, const double &start_x,
    const double &start_y, const double &end_x, const double &end_y,
    const int &cycles) {
  cells_visited_ = 0;
  threshold_ = lethal_cost_;

  currentEnd_ = 0;
  nextEnd_ = 0;
  overEnd_ = 0;

  memset(pending_.get(), false, ns_ * sizeof(bool));
  std::fill(potential_.get(), potential_.get() + ns_, POT_HIGH);
  unsigned int k = toIndex(start_x, start_y);
  LOG(INFO) << "start_grid x: " << start_x << " y: " << start_y;
  if (k > (ns_ - nx_ * 2 - 1) || k < nx_ + 1) {
    LOG(ERROR) << "Start Pose k index is over Map.";
    return false;
  }
  double dx = start_x - floorf(start_x);
  double dy = start_y - floorf(start_y);
  // 更新起点附近的势场值
  dx = floorf(dx * 100 + 0.5) / 100;
  dy = floorf(dy * 100 + 0.5) / 100;
  potential_[k] = neutral_cost_ * 2 * dx * dy;
  potential_[k + 1] = neutral_cost_ * 2 * (1 - dx) * dy;
  potential_[k + nx_] = neutral_cost_ * 2 * dx * (1 - dy);
  potential_[k + nx_ + 1] = neutral_cost_ * 2 * (1 - dx) * (1 - dy);

  // 将起点旁边各方向的节点加入到优先级队列，准备循环遍历计算整个势场地图
  pushCur(costs, k + 2);
  pushCur(costs, k - 1);
  pushCur(costs, k + nx_ - 1);
  pushCur(costs, k + nx_ + 2);

  pushCur(costs, k - nx_);
  pushCur(costs, k - nx_ + 1);
  pushCur(costs, k + nx_ * 2);
  pushCur(costs, k + nx_ * 2 + 1);

  int current_index = 0;
  int cycle = 0;  // 循环次数
  int startCell = toIndex(end_x, end_y);
  auto pb = currentBuffer_.get();

  for (; cycle < cycles; cycle++) {
    if (currentEnd_ == 0 && nextEnd_ == 0) {
      LOG(ERROR) << "currentEnd_ nextEnd_ == 0.";
      return false;
    }
    // 根据当前优先级队列重置标志位队列
    pb = currentBuffer_.get();
    current_index = currentEnd_;

    while (current_index-- > 0) { pending_[*(pb++)] = false; }

    pb = currentBuffer_.get();
    current_index = currentEnd_;
    while (current_index-- > 0) { updateCell(costs, *pb++); }
    currentEnd_ = nextEnd_;
    nextEnd_ = 0;
    currentBuffer_.swap(nextBuffer_);

    if (currentEnd_ == 0) {
      threshold_ += priorityIncrement_;
      currentEnd_ = overEnd_;
      overEnd_ = 0;
      currentBuffer_.swap(overBuffer_);
    }

    if (potential_[startCell] < POT_HIGH) {
      break;
    }
  }
  if (cycle < cycles) {
    return true;
  } else {
    LOG(ERROR) << "cycle over.";
    return false;
  }
}

bool DijkstraExpansion::calculateAllPotentials(
    const boost::shared_array<unsigned char> &costs, const double &start_x,
    const double &start_y, const double &end_x, const double &end_y,
    const int &cycles, int lethal_cost) {
  cells_visited_ = 0;
  threshold_ = lethal_cost_ = lethal_cost;
  // threshold_ = 130;
  // lethal_cost_ = 130;
  factor_ = 1.0;
  neutral_cost_ = 10;

  currentEnd_ = 0;
  nextEnd_ = 0;
  overEnd_ = 0;

  memset(pending_.get(), false, ns_ * sizeof(bool));
  std::fill(potential_.get(), potential_.get() + ns_, POT_HIGH);
  unsigned int k = toIndex(start_x, start_y);
  int startCell = toIndex(end_x, end_y);
  if (k > (ns_ - nx_ * 2 - 1) || k < nx_ + 1) {
    LOG(ERROR) << "Start Pose k index is over Map.";
    return false;
  }
  double dx = start_x - floorf(start_x);
  double dy = start_y - floorf(start_y);
  // 更新起点附近的势场值
  dx = floorf(dx * 100 + 0.5) / 100;
  dy = floorf(dy * 100 + 0.5) / 100;
  potential_[k] = neutral_cost_ * 2 * dx * dy;
  potential_[k + 1] = neutral_cost_ * 2 * (1 - dx) * dy;
  potential_[k + nx_] = neutral_cost_ * 2 * dx * (1 - dy);
  potential_[k + nx_ + 1] = neutral_cost_ * 2 * (1 - dx) * (1 - dy);

  // 将起点旁边各方向的节点加入到优先级队列，准备循环遍历计算整个势场地图
  pushCur(costs, k + 2);
  pushCur(costs, k - 1);
  pushCur(costs, k + nx_ - 1);
  pushCur(costs, k + nx_ + 2);

  pushCur(costs, k - nx_);
  pushCur(costs, k - nx_ + 1);
  pushCur(costs, k + nx_ * 2);
  pushCur(costs, k + nx_ * 2 + 1);

  int current_index = 0;
  int cycle = 0;  // 循环次数
  auto pb = currentBuffer_.get();

  for (; cycle < cycles; cycle++) {
    if (currentEnd_ == 0 && nextEnd_ == 0) {
      LOG(ERROR) << "currentEnd_ nextEnd_ == 0.";
      // return false;
      break;
    }
    // 根据当前优先级队列重置标志位队列
    pb = currentBuffer_.get();
    current_index = currentEnd_;

    while (current_index-- > 0) { pending_[*(pb++)] = false; }

    pb = currentBuffer_.get();
    current_index = currentEnd_;
    while (current_index-- > 0) { updateCell(costs, *pb++); }
    currentEnd_ = nextEnd_;
    nextEnd_ = 0;
    currentBuffer_.swap(nextBuffer_);

    if (currentEnd_ == 0) {
      threshold_ += priorityIncrement_;
      currentEnd_ = overEnd_;
      overEnd_ = 0;
      currentBuffer_.swap(overBuffer_);
    }
  }
  LOG(INFO) << "potential_[startCell]: " << potential_[startCell];
  if (potential_[startCell] < POT_HIGH - 1.0) {
    return true;
  } else {
    return false;
  }
}

void DijkstraExpansion::calculateAllPotentials(
    const boost::shared_array<unsigned char> &costs, const double &start_x,
    const double &start_y, const int &cycles, int lethal_cost) {
  cells_visited_ = 0;
  threshold_ = lethal_cost_ = lethal_cost;
  // threshold_ = lethal_cost_;
  // threshold_ = 130;
  // lethal_cost_ = 130;
  factor_ = 1.0;
  neutral_cost_ = 10;

  currentEnd_ = 0;
  nextEnd_ = 0;
  overEnd_ = 0;

  memset(pending_.get(), false, ns_ * sizeof(bool));
  std::fill(potential_.get(), potential_.get() + ns_, POT_HIGH);
  unsigned int k = toIndex(start_x, start_y);
  if (k > (ns_ - nx_ * 2 - 1) || k < nx_ + 1) {
    LOG(ERROR) << "Start Pose k index is over Map.";
    return;
  }
  double dx = start_x - floorf(start_x);
  double dy = start_y - floorf(start_y);
  // 更新起点附近的势场值
  dx = floorf(dx * 100 + 0.5) / 100;
  dy = floorf(dy * 100 + 0.5) / 100;
  potential_[k] = neutral_cost_ * 2 * dx * dy;
  potential_[k + 1] = neutral_cost_ * 2 * (1 - dx) * dy;
  potential_[k + nx_] = neutral_cost_ * 2 * dx * (1 - dy);
  potential_[k + nx_ + 1] = neutral_cost_ * 2 * (1 - dx) * (1 - dy);

  // 将起点旁边各方向的节点加入到优先级队列，准备循环遍历计算整个势场地图
  pushCur(costs, k + 2);
  pushCur(costs, k - 1);
  pushCur(costs, k + nx_ - 1);
  pushCur(costs, k + nx_ + 2);

  pushCur(costs, k - nx_);
  pushCur(costs, k - nx_ + 1);
  pushCur(costs, k + nx_ * 2);
  pushCur(costs, k + nx_ * 2 + 1);

  int current_index = 0;
  int cycle = 0;  // 循环次数
  auto pb = currentBuffer_.get();

  for (; cycle < cycles; cycle++) {
    if (currentEnd_ == 0 && nextEnd_ == 0) {
      LOG(ERROR) << "currentEnd_ nextEnd_ == 0.";
      // return false;
      break;
    }
    // 根据当前优先级队列重置标志位队列
    pb = currentBuffer_.get();
    current_index = currentEnd_;

    while (current_index-- > 0) { pending_[*(pb++)] = false; }

    pb = currentBuffer_.get();
    current_index = currentEnd_;
    while (current_index-- > 0) { updateCell(costs, *pb++); }
    currentEnd_ = nextEnd_;
    nextEnd_ = 0;
    currentBuffer_.swap(nextBuffer_);

    if (currentEnd_ == 0) {
      threshold_ += priorityIncrement_;
      currentEnd_ = overEnd_;
      overEnd_ = 0;
      currentBuffer_.swap(overBuffer_);
    }
  }
  return;
}

bool DijkstraExpansion::makePlan(boost::shared_array<unsigned char> &costs,
                                 const Pose2d &start_pose,
                                 const Pose2d &end_pose,
                                 std::vector<Pose2d> &path_result) {
  // 254是 costmap里面的LETHAL_OBSTACLE = 254
  // 注意到时候合并对接新接口时该值的修改
  if (outlineMap(costs, nx_, ny_, 254)) {
    if (calculatePotentials(costs, start_pose.x, start_pose.y, end_pose.x,
                            end_pose.y, nx_ * ny_ * 2)) {
      if (getPath(start_pose.x, start_pose.y, end_pose.x, end_pose.y,
                  path_result)) {
        if (!path_result.empty()) {
          LOG(INFO) << "makePlan succed with size = " << path_result.size();
          return true;
        } else {
          LOG(INFO) << "Too close, make an empty path.";
          return false;
        }
      } else {
        LOG(ERROR) << "[DijkstraExpansion] Get Path From Potential Map Failed.";
        return false;
      }
    } else {
      LOG(ERROR) << "[DijkstraExpansion] Calc Map's Potential Failed.";
      return false;
    }
  } else {
    LOG(ERROR) << "[DijkstraExpansion] Can't outline the map.";
    return false;
  }
}

}  // namespace CVTE_BABOT