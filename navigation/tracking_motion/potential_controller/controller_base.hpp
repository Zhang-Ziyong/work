/*
 * @Author: your name
 * @Date: 2020-10-29 16:53:36
 * @LastEditTime: 2020-11-24 15:38:34
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/potential_controller/controller_base.hpp
 */
#ifndef CONTROLLER_BASE_HPP_
#define CONTROLLER_BASE_HPP_
#include "slam_math.hpp"
#include <vector>
#include "data_struct.hpp"
#include "tracking_motion_config.hpp"
namespace TRACKING_MOTION {
class ControllerBase {
 public:
  virtual ~ControllerBase() {}
  ControllerBase() {}
  virtual void computeCommand(CmdVel &cmd_vel, const Vec2d &predict_vel) = 0;
  virtual void setTargetPose(const Mat34d &target_pose) = 0;
  virtual void InputScan(const std::vector<Vec2d> &scan) = 0;
  virtual void setCurVel(double vel) = 0;

 private:
};
}  // namespace TRACKING_MOTION
#endif