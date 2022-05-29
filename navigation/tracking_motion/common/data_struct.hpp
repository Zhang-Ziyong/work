/*
 * @Author: your name
 * @Date: 2020-10-29 17:46:29
 * @LastEditTime: 2020-11-04 16:45:07
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/data_struct/data_struct.hpp
 */
#ifndef DATA_STRUCT_HPP_
#define DATA_STRUCT_HPP_
namespace TRACKING_MOTION {
struct CmdVel {
  CmdVel() {}
  CmdVel(double vel, double w) : vel_(vel), w_(w) {}
  double vel_;
  double w_;
};
}  // namespace TRACKING_MOTION
#endif