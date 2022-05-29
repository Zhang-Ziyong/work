/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file global_planner.hpp
 *
 *@brief 对通用或其他全局规划算法的二次封装
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-18
 ************************************************************************/

#ifndef __GLOBAL_PLANNER_HPP
#define __GLOBAL_PLANNER_HPP

#include "costmap_2d.hpp"
#include "costmap_utils.hpp"
#include "dijkstra/D_star.hpp"
#include "hybrid/hybrid_a_star.hpp"
#include "kinodynamic/kinodynamic_astar.hpp"
#include "bsline/path_optimizer.hpp"
#include <glog/logging.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include "path.hpp"
namespace CVTE_BABOT {

class GlobalPlanner {
 public:
  GlobalPlanner() = default;
  GlobalPlanner(const GlobalPlanner &obj) = delete;
  GlobalPlanner &operator=(const GlobalPlanner &obj) = delete;
  virtual ~GlobalPlanner() = default;

  /**
   * makePlan
   * @brief
   *   输入一张地图，根据起点和终点规划出一条最短路径
   *   所有规划算法都应该提供这个规划的接口
   *
   * @param[in] costs-输入的代价地图
   * @param[in] start_x-起点x值
   * @param[in] start_y-起点y值
   * @param[in] end_x-终点x值
   * @param[in] end_y-终点y值
   * @param[out] n-二维坐标对应一维的索引点
   * @ return true-代表规划路径成功
   * */
  virtual bool makePlan(boost::shared_array<unsigned char> &, const Pose2d &,
                        const Pose2d &, SubPath &){};

  /**
   * makePlan
   * @brief
   *   对上面makePlan的封装，包含costmap中的地图与图片坐标转换和路径结果保存
   *
   * @param[in] MakePlanInfo-规划路径的信息结构体
   * @param[out] n-二维坐标对应一维的索引点
   * @ return true-代表规划路径成功
   * */
  virtual bool makePlan(const MakePlanInfo &info, SubPath &path_result);

  /**
   * calcPathPointAngle
   * @brief
   *   通过两点之间的朝向，计算路径中的角度信息
   *
   * @param[in]
   * path_result-规划算法计算得到的路径（若是用D星算法则路径中角度为零）
   * @param[in] target-路径规划的目标点
   * */
  void calcPathPointAngle(std::vector<Pose2d> &path_result,
                          const Pose2d &target);
  /**
   * @brief 路径插值平滑
   *
   * @param origin_path
   * @param smooth_path
   */
  void smoothPath(const std::vector<Pose2d> &origin_path,
                  std::vector<Pose2d> &smooth_path);

  /**
   * pathPointToWorld
   * @brief
   *   地图坐标系转世界坐标系
   *
   * @param[in] d_mx-costmap内起点的x值
   * @param[in] d_my-costmap内起点的y值
   * @param[in] d_wx-世界坐标系下的x值
   * @param[in] d_wy-世界坐标系下的y值
   * */
  void pathPointToWorld(const double &d_mx, const double &d_my, double &d_wx,
                        double &d_wy);

  /**
   * updateCostMap
   * @brief
   *   更新需要规划的costmap
   *   切记！规划前要将当前的costmap更新进来
   *
   * @param[in] costmap-当前costmap的指针
   * */
  inline void updateCostMap(std::shared_ptr<Costmap2d> costmap) {
    gb_planner_costmap_ptr_ = costmap;
  }

  std::shared_ptr<Costmap2d> getCostmap() { return gb_planner_costmap_ptr_; }

 private:
  std::shared_ptr<Costmap2d> gb_planner_costmap_ptr_ = nullptr;
};

class DijkstraGlobalPlanner : public GlobalPlanner {
 public:
  DijkstraGlobalPlanner(const DijkstraGlobalPlanner &obj) = delete;
  DijkstraGlobalPlanner &operator=(const DijkstraGlobalPlanner &obj) = delete;
  ~DijkstraGlobalPlanner() = default;

  DijkstraGlobalPlanner(const unsigned int &nx, const unsigned int &ny) {
    dijkstra_gb_planner_ptr_ = std::make_shared<DijkstraExpansion>(nx, ny);
  };

  bool makePlan(boost::shared_array<unsigned char> &costs,
                const Pose2d &start_pose, const Pose2d &end_pose,
                SubPath &path_result) override {
    return dijkstra_gb_planner_ptr_->makePlan(costs, start_pose, end_pose,
                                              path_result.wps);
  }

 private:
  std::shared_ptr<DijkstraExpansion> dijkstra_gb_planner_ptr_;
};

class HybridGlobalPlanner : public GlobalPlanner {
 public:
  HybridGlobalPlanner(const HybridGlobalPlanner &obj) = delete;
  HybridGlobalPlanner &operator=(const HybridGlobalPlanner &obj) = delete;
  ~HybridGlobalPlanner() = default;

  HybridGlobalPlanner(const unsigned int &nx, const unsigned int &ny) {
    hybrid_gb_planner_ptr_ = std::make_shared<HybridAStartPlanner>(nx, ny);
  };

  bool makePlan(boost::shared_array<unsigned char> &costs,
                const Pose2d &start_pose, const Pose2d &end_pose,
                SubPath &path_result) override {
    return hybrid_gb_planner_ptr_->makePlan(costs, start_pose, end_pose,
                                            path_result.wps);
  }

 private:
  std::shared_ptr<HybridAStartPlanner> hybrid_gb_planner_ptr_;
};

class KinodynamicGlobalPlanner : public GlobalPlanner {
 public:
  KinodynamicGlobalPlanner(const KinodynamicGlobalPlanner &obj) = delete;
  KinodynamicGlobalPlanner &operator=(const KinodynamicGlobalPlanner &obj) =
      delete;
  KinodynamicGlobalPlanner() {
    kinodynamic_gb_planner_ptr_ = std::make_shared<KinodynamicAstar>();
  };

  ~KinodynamicGlobalPlanner() = default;

  bool makePlan(const MakePlanInfo &info, SubPath &path_result) override {
    Eigen::Vector3d start_pt(info.start_point.getX(), info.start_point.getY(),
                             info.start_point.getYaw());
    Eigen::Vector3d end_pt(info.target_point.getX(), info.target_point.getY(),
                           info.target_point.getYaw());
    kinodynamic_gb_planner_ptr_->reset();
    kinodynamic_gb_planner_ptr_->setCostMap(getCostmap());
    kinodynamic_gb_planner_ptr_->init("global");
    LOG(INFO) << "Make kinodynamic path.";
    bool init =
        info.start_vel(0) > 0.4 ? true : false;  // 判断是否具有初速度的规划
    if (kinodynamic_gb_planner_ptr_->search(
            start_pt, info.start_vel, end_pt, info.target_vel, init, false,
            -1.0) == KinodynamicAstar::REACH_END) {
      path_result = kinodynamic_gb_planner_ptr_->getKinoTraj(0.2);

      for (auto p : path_result.wps) {
        LOG(INFO) << p.getX() << ", " << p.getY() << ", " << p.getYaw();
      }

      for (auto p : path_result.wpis) {
        LOG(INFO) << p.v << ", " << p.w << ", " << p.curve;
      }

      return true;
    }
    return false;
  }

 private:
  std::shared_ptr<KinodynamicAstar> kinodynamic_gb_planner_ptr_ = nullptr;
};

}  // namespace CVTE_BABOT

#endif  // end of __GLOBAL_PLANNER_HPP