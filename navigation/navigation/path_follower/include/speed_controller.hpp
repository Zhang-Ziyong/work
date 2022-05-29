/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file speed_controller.hpp
 *
 *@brief 有关导航速度等级变换的控制类
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-06-09
 ************************************************************************/

#ifndef __SPEED_CONTROLLER_HPP
#define __SPEED_CONTROLLER_HPP

#include <memory>
#include <mutex>
#include "costmap_utils.hpp"
#include "planner_utils.hpp"
namespace CVTE_BABOT {

struct SpeedCtrlRes {
  double x_vel = 0.0;
  double y_vel = 0.0;
  double th_vel = 0.0;
  std::string stop_by_who = "";
};

class SpeedController final {
 public:
  ~SpeedController() = default;

  /**
   *getPtrInstance
   *@brief
   *获取当前的单例指针
   **/
  static std::shared_ptr<SpeedController> getPtrInstance();

  /**
  *setMaxVelocity
  *@brief
  *  设置导航允许的最大速度
  *
  *@param[in] v_x - X轴方向最大线速度
  *@param[in] v_y - Y轴方向最大线速度
  *@param[in] v_th - 最大角速度
  **/
  inline void setMaxVelocity(const double &v_x, const double &v_y,
                             const double &v_th) {
    max_velocity_.d_x = v_x;
    max_velocity_.d_y = v_y;
    max_velocity_.d_yaw = v_th;
  }

  /**
  *setSpeedResultPercent
  *@brief
  *  按照比例修改导航速度，4档（100%）代表使用最大线速度执行导航速度计算
  *  最大线速度通过上面setMaxVelocity的接口修改，目前默认为 1 m/s
  *  -------------------------
  *  目前线速度和角速度的百分比默认为四档（25%， 50%， 75%， 100%）
  *
  *@param[out] speed_res - 最大速度值按照百分比修改后的结果
  *@param[in] vel_percent - 线速度的百分比
  *@param[in] th_percent - 角速度的百分比
  **/
  inline void setSpeedResultPercent(SpeedCtrlRes &speed_res,
                                    const double &vel_percent,
                                    const double &th_percent);

  /**
  *adjustSpeedLevel
  *@brief
  *  外部调用的主要接口函数，实时获取目前需要执行导航的速度档位
  *  内部会综合 定位/任务管理/costmap三者的速度来判断，取速度
  *  档位最小的那个来执行
  *  ------------------------------------------------
  *  当三者中某一返回速度等级为-1时，代表需要马上停车，此函数会
  *  返回是哪个部分触发停止，供外部反馈状态使用
  *
  *@param[out] speed_res - 最终执行的速度以及是由于哪部分触发停车
  *@return true - 代表目前三者中有一个或以上读到-1的档位，需要停车
  *        false - 代表可以正常按照速度档位导航
  **/
  bool adjustSpeedLevel(SpeedCtrlRes &speed_res);

  /**
  *calcMissionMangerSpeedLevel
  *@brief
  *  计算任务模块设置的导航速度
  *  主要是根据int型值大小，映射为对应的速度档位枚举型
  *
  *@param[in] level - 任务模块设置的导航档位（支持 1\2\3\4）
  **/
  void calcMissionMangerSpeedLevel(const unsigned int &level);

  /**
  *calcMarkSpeedLevel
  *@brief
  *  设置区域内的速度等级
  *
  *@param[in] level - 区域内的导航档位（支持 1\2\3\4）
  **/
  void calcMarkSpeedLevel(SpeedLevel level);

  /**
  *calcLocalizaitonSpeedLevel
  *@brief
  *  计算任务模块设置的导航速度
  *  主要是根据int型值大小，映射为对应的速度档位枚举型
  *
  *@param[in] level - 任务模块设置的导航档位（支持 1\2\3\4）
  **/
  void calcLocalizaitonSpeedLevel(const double &cov);

  /**
  *intToSpeedLevel
  *@brief
  *  根据int型值大小，映射为对应的速度档位枚举型
  *
  *@param[in] level - 任务模块设置的导航档位（支持 1\2\3\4）
  **/
  SpeedLevel intToSpeedLevel(const unsigned int &level);

  /**
  *getCostmapSpeedLevel
  *getLocalizationSpeedLevel
  *getMissionManagerSpeedLevel
  *@brief
  *  获取四个模块的当前速度档位
  *
  *@return SpeedLevel速度档位的枚举类型
  **/
  SpeedLevel getCostmapSpeedLevel();
  SpeedLevel getLocalizationSpeedLevel();
  SpeedLevel getMissionManagerSpeedLevel();
  SpeedLevel getMarkSpeedLevel();

 private:
  SpeedController();
  SpeedController(const SpeedController &obj) = delete;
  SpeedController &operator=(const SpeedController &obj) = delete;

  static std::shared_ptr<SpeedController> speed_ctrl_ptr_;

  unsigned int adjust_level_count_ = 0;  ///< 速度等级之间切换的时间计数位

  Velocity max_velocity_;  ///< 记录导航参数设置的最大速度

  ///< 当前的导航速度等级
  SpeedLevel current_level_ = SpeedMax;
  ///< costmap中根据激光和超声决定的导航速度等级
  SpeedLevel costmap_speed_level_ = SpeedMax;
  ///< 任务管理下发的导航速度等级
  SpeedLevel mission_manager_level_ = SpeedMax;
  ///< 定位可信度决定的导航速度等级
  SpeedLevel localization_speed_level_ = SpeedMax;
  ///< 区域标记控制的速度等级
  SpeedLevel mark_speed_level_ = SpeedMax;

  std::mutex localization_speed_level_mutex_;  ///< 对应上面三个速度等级的锁
  std::mutex costmap_speed_level_mutex_;  ///< ---------------------
  std::mutex mission_manager_level_mutex_;  ///< 防止外面动态设置导致访问冲突
  std::mutex mark_speed_level_mutex_;  ///< 防止外面动态设置导致访问冲突
};

}  // namespace CVTE_BABOT

#endif  // __SPEED_CONTROLLER_HPP