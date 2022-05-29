/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file path.hpp
 *
 *@brief 导航路径类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-16
 ************************************************************************/
#ifndef __PATH_HPP
#define __PATH_HPP

#include "pose2d/pose2d.hpp"
#include "glog/logging.h"
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <eigen3/Eigen/Core>

namespace CVTE_BABOT {

// 标识路径属性【控制路径、索引路径、局部路径、普通路径（默认）】
enum PathType { CTROL, REFER, LOCAL, NORMAL };

struct WayPointInfo {
  WayPointInfo(){};

  WayPointInfo(double vv, double ww, double cc) {
    this->v = vv;
    this->w = ww;
    this->curve = cc;
    this->is_edge_wise = 0;
    double edge_dist = 0.0;
  }
  WayPointInfo(double vv, double ww, double cc, int _is_edge_wise,
               double _edge_dist)
      : v(vv),
        w(ww),
        curve(cc),
        is_edge_wise(_is_edge_wise),
        edge_dist(_edge_dist) {}
  double v = 0.0;
  double w = 0.0;
  double curve = 0.0;
  int is_edge_wise = 0;
  double edge_dist;        //贴边距离: 左边为正,右边为负
  double path_length = 0;  // 记录到起点的路径长度总和
};

typedef struct {
  Pose2d start_point;
  Pose2d target_point;
  Eigen::Vector3d start_vel;
  Eigen::Vector3d target_vel;
  double path_sample = 0.2;
} MakePlanInfo;

struct PoseMsg {
  Pose2d position;
  int index = 0;
  WayPointInfo velocity;
};

struct SubPath {
  SubPath(bool forward = true) : forward(forward) {}
  SubPath(const SubPath &obj) {
    this->wpis = obj.wpis;
    this->wps = obj.wps;
    this->forward = obj.forward;
    this->type = obj.type;
  }

  std::size_t size() const { return wps.size(); }

  void clear() {
    wps.clear();
    wpis.clear();
  }

  bool empty() const { return wps.empty(); }

  std::vector<Pose2d>::iterator begin() { return wps.begin(); }
  std::vector<Pose2d>::const_iterator begin() const { return wps.begin(); }
  std::vector<Pose2d>::iterator end() { return wps.end(); }
  std::vector<Pose2d>::const_iterator end() const { return wps.end(); }

  bool info_empty() const { return wpis.empty(); }
  std::vector<WayPointInfo>::iterator info_begin() {
    if (wpis.size() == wps.size()) {
      return wpis.begin();
    } else {
      return wpis.end();
    }
  }
  std::vector<WayPointInfo>::const_iterator info_begin() const {
    if (wpis.size() == wps.size()) {
      return wpis.begin();
    } else {
      return wpis.end();
    }
  }
  std::vector<WayPointInfo>::iterator info_end() { return wpis.end(); }
  std::vector<WayPointInfo>::const_iterator info_end() const {
    return wpis.end();
  }

  Pose2d front() { return wps.front(); }
  Pose2d back() { return wps.back(); }
  Pose2d operator[](const std::size_t &i) { return wps[i]; }
  Pose2d at(const std::size_t &i) { return wps.at(i); }

  /// a method to add a SubPath to a SubPath
  SubPath operator+(const SubPath &b) const {
    SubPath path(*this);
    if (b.wps.size() > 0) {
      path.wps.insert(path.wps.end(), b.wps.begin(), b.wps.end());
      path.wpis.insert(path.wpis.end(), b.wpis.begin(), b.wpis.end());
    }
    return path;
  }

  void push_back(const Pose2d &wp) { wps.push_back(wp); }
  void push_back(const WayPointInfo &wi) { wpis.push_back(wi); }

  template <typename... Args>
  void emplace_back(Args &&... args) {
    wps.emplace_back(std::forward<Args>(args)...);
  }

  void reverseCurrentPath(int sidx, int eidx) {
    // 提取轨迹位姿和信息
    if (wps.size() != wpis.size() || sidx < 0 || sidx >= wps.size()) {
      LOG(WARNING) << "input idx: [" << sidx << ", " << eidx
                   << "] or poses size " << int(wps.size()) << " isnt valid !";
      return;
    }

    // 取反给定区间内的位姿点
    for (int i = sidx; i <= eidx; i++) {
      // 取反姿态
      wps[i].yaw += M_PI;
      if (wps[i].yaw > M_PI)
        wps[i].yaw -= 2 * M_PI;
      // 取反速度
      wpis[i].v = -wpis[i].v;
    }
  }

  std::vector<Pose2d> wps;
  std::vector<WayPointInfo> wpis;
  bool forward;
  PathType type = PathType::NORMAL;
};

class Path {
 public:
  typedef std::shared_ptr<Path> Ptr;
  typedef std::shared_ptr<Path const> ConstPtr;
  typedef std::function<void()> NextWayPointCallback_t;

  Path() : has_callback_(false) {
    path_ = std::vector<SubPath>();
    path_.push_back(SubPath());
    current_sub_path_ = path_.begin();
  }

  // Path(const Path &) = delete;
  // Path &operator=(const Path &) = delete;

  void clear();
  void reset();
  void setPath(const std::vector<SubPath> &path);
  void registerNextWayPointCallback(NextWayPointCallback_t func);
  void switchToNextSubPath();
  void switchToNextWayPoint();
  void clearNewPathFlag();
  void fireNextWayPointCallback();
  void setPathTargetPoint(const Pose2d &point);

  bool empty() const;
  bool isDone() const;
  bool isSubPathDone() const;
  bool isLastWayPoint() const;
  bool isNewPath() const;

  std::shared_ptr<SubPath> getCurrentSubPath() const;
  std::shared_ptr<SubPath> getSubPath(size_t idx) const;
  Pose2d getWayPoint(size_t idx) const;
  Pose2d getCurrentWayPoint() const;
  Pose2d getLastWayPoint() const;
  Pose2d getPathTargetPoint() const;

  size_t getWayPointIndex() const;
  size_t subPathCount() const;

  float getRemainingSubPathDistance() const;
  double getCurrentPathAngle() const;

  /**
   *reverseCurrentPath
   *@brief
   *反转部分轨迹，使得机器人可以反向跟踪这段轨迹
   *
   *@param[in] sidx
   *起点索引
   *
   *@param[in] eidx
   *终点索引
   *
   **/
  // void reverseCurrentPath(int sidx, int eidx);

 private:
  std::vector<float> wp_distance_to_end_;
  std::vector<SubPath> path_;
  std::vector<SubPath>::iterator current_sub_path_;
  Pose2d target_point;  // 用来记录这条路径的目标点

  size_t next_waypoint_idx_ = 0;
  NextWayPointCallback_t next_wp_callback_;

  bool has_callback_ = false;
  bool new_path_flag_ = false;

  void computeWayPointToEndDistances();
};

}  // namespace CVTE_BABOT

#endif
