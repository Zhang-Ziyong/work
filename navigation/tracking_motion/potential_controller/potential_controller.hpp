/*
 * @Author: your name
 * @Date: 2020-10-30 15:43:47
 * @LastEditTime: 2020-11-24 15:38:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/potential_controller/potential_controller.hpp
 */
#ifndef POTENTIAL_CONTROLLER_HPP_
#define POTENTIAL_CONTROLLER_HPP_
#include "controller_base.hpp"
#include "tracking_motion_config.hpp"
#include "slam_math.hpp"
#include <mutex>
#include <thread>
namespace TRACKING_MOTION {
class PotentialController final : public ControllerBase {
 public:
  PotentialController(const PotentialControllerConfig &config);
  ~PotentialController();
  PotentialController(const PotentialController &obj) = delete;
  const PotentialController &operator=(const PotentialController &obj) = delete;
  virtual void computeCommand(CmdVel &cmd_vel,
                              const Vec2d &predict_vel) override;
  virtual void setTargetPose(const Mat34d &target_pose) override;
  virtual void setCurVel(double vel) override;
  virtual void InputScan(const std::vector<Vec2d> &scan) override;

 private:
  void obstackeStopThreadCallback();
  Vec3d computeAttractiveForce();
  Vec3d computeRepulsiveForce();
  bool isPointInRepulsive(const Vec2d &point);
  bool isPointInObstacleStop(const Vec2d &point);
  bool pointInStopPosLinear(const Vec2d &point);
  bool pointInStopNegtiveLinear(const Vec2d &point);
  bool pointInStopPosRot(const Vec2d &point);
  bool pointInStopNegtiveRot(const Vec2d &point);

  std::thread check_obstacle_stop_thread_;

  PotentialControllerConfig config_;
  Mat34d target_pose_;
  double cur_vel_;
  std::vector<Vec2d> scan_;

  std::mutex target_pose_mutex_;
  std::mutex scan_mutex_;

  bool is_stop_;
  bool is_obstacle_stop_;
};
}  // namespace TRACKING_MOTION

#endif