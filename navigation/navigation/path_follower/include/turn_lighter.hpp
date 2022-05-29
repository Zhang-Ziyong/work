/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file turn_lighter.hpp
 *
 *@brief 负责更新转向灯，提供查询接下来需要往哪边转
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev
 *@data 2021 - 02 - 26
 ************************************************************************/

#ifndef __TURN_LIGHTER_HPP
#define __TURN_LIGHTER_HPP

#include "path.hpp"

namespace CVTE_BABOT {

class TurnLighter {
public:
  TurnLighter() = default;
  ~TurnLighter() = default;

  /**
   *updateTurnLightWithCtrlPath
   *@brief
   *  将当前控制路径的末位点，投影到机器人坐标系，看是在左边还是右边，进行打灯
   **/
  void updateTurnLightWithCtrlPath(const SubPath &ctrl_path);

  /**
   *updateTurnLightWithRotate
   *@brief
   *  根据当前原地旋转的速度方向，来决定打灯方向
   **/
  void updateTurnLightWithRotate(const double &w_vel);

  inline void resetForwardTurnLight() {
    control_direction_ = 0.0;
  }
  
  /**
   *getCtrlDirection
   *@brief
   *  获取控制器当前方向，用于外部控制方向灯
   *
   *@return -1 - 代表准备右转
   *         1 - 代表准备左转
   *         0 - 代表不用转弯
   **/
  int getCtrlDirection();
  int getCtrlDirectionWithFilter(); // 带滤波

private:
  double control_direction_ = 0.0;
  double turn_direction_tolerance_ = 0.5;
  unsigned int direction_count_ = 0;
  int last_control_direction_ = 0;
  int final_control_direction_ = 0;
};

}


#endif // end of __TURN_LIGHTER_HPP