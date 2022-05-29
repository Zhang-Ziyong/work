/*
 * @Author: your name
 * @Date: 2021-09-28 16:37:16
 * @LastEditTime: 2021-09-28 17:49:50
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /navigation/navigation/clean_decision/config/clean_decision_config.hpp
 */
#ifndef CLEAN_DECISION_CONFIG_HPP_
#define CLEAN_DECISION_CONFIG_HPP_

namespace CVTE_BABOT {
// 任务类型
//　清洁配置参数
struct CleanConfig {
  CleanConfig()
      : clean_width(0.6),
        clean_length(0.5),
        update_thresh(0.3),
        min_box_length(2.0) {}
  CleanConfig(double _clean_width, double _update_thresh, double _clean_length,
              double _min_box_length)
      : clean_width(_clean_width),
        clean_length(_clean_length),
        update_thresh(_update_thresh),
        min_box_length(_min_box_length) {}
  bool auto_replan;
  bool auto_supplement_clean;
  bool auto_unreach_clean;
  double clean_width;     // 清扫框宽度
  double clean_length;    // 清扫框长
  double update_thresh;   //更新清扫区域定位变化阈值
  double min_box_length;  //最小清扫框
};
}  // namespace CVTE_BABOT

#endif
