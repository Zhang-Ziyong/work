/*
 * @Author: linyanlong@cvte.com
 * @Date: 2020-07-14 19:31:16
 * @LastEditTime: 2020-07-29 19:19:17
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /cvte_lidar_slam/include/state_machine/slam_state_machine.hpp
 */
#ifndef SLAM_STATE_MACHINE_HPP
#define SLAM_STATE_MACHINE_HPP
#include <functional>
#include <vector>
#include <map>
#include <mutex>
#include <string>
#include "stateMachine.hpp"
#include "memory"
namespace cvte_lidar_slam {
enum eventTypes {
  LIDAR_SLAM_EVENT,
};

enum EventName {
  STOP_LOCALIZAION,
  START_LOCALIZATION,
  INIT_LOCALIZATION_SUCCESS,
  SAVE_MAP,
  START_MAPPING,
  CANCLE_MAPPING,
  RETURN_MAPPING,
  STOP,
  GET_STATE,
  SET_PARTOL_POINT
};

enum AD_MODE { AD_UNKNOW, AD_MAPPING, AD_LOCALIZATION };

enum AD_STATU { UNKNOW, LOCALIZATION, INIT_LOCALIZATION, MAPPING };

enum SENSOR_STATUS {
  NORMAL,
  ODOM_DELAY,
  ODOM_TIME_STAMP_JUMP,
  ODOM_POSE_JUMP,
  LASER_DELAY
};

struct eventPayload {
  eventPayload(){};
  eventPayload(const std::string &condition, const std::string &expected_state)
      : condition_(condition), expected_state_(expected_state) {}

  std::string condition_;       ///< 事件payload的名字（状态转移名）
  std::string expected_state_;  ///< 事件期望状态
};

class SlamStateMachine {
 public:
  static std::shared_ptr<SlamStateMachine> getInstance();
  /**
   *registerTransActionCallback
   *@brief
   *注册状态转移回调函数
   *@return  bool
   **/
  bool registerTransActionCallback(const EventName &event_name,
                                   std::function<bool()> p_call_back);
  /**
   *registerEntryActionCallback
   *@brief
   *注册状态进入回调函数
   *@return  bool
   **/
  bool registerEntryActionCallback(const AD_STATU &state_name,
                                   std::function<void()> p_callback);
  /**
   *sendEvent
   *@brief
   *发送事件
   *@return  bool
   **/
  bool sendEvent(const EventName &even_name);
  /**
   *getCurrentStateName
   *@brief
   *获得当前状态名字
   *@return  状态名字
   **/
  std::string getCurrentStateName();
  /**
   *getCurrentModeName
   *@brief
   *获得当前模式名字
   *@return  模式名字
   **/
  std::string getCurrentModeName();
  /**
   *getCurrentState
   *@brief
   *获得当前状态
   *@return  状态
   **/
  AD_STATU getCurrentState();
  /**
   *getCurrentMode
   *@brief
   *获得当前模式
   *@return  模式
   **/
  AD_MODE getCurrentMode();

  AD_STATU setCurrentState(AD_STATU state);

  AD_MODE setCurrentMode(AD_MODE mode);

 private:
  void initState();
  /**
   *registerState
   *@brief
   *注册状态
   *@return
   **/
  void registerState(state *slam_state, state *p_entry_state = nullptr);
  /**
   *registerTransition
   *@brief
   *注册状态转移
   *@return
   **/
  void registerTransition(transition &trans, std::string *condition,
                          state *next_state);
  /**
   *transAction
   *@brief
   *状态转移回调
   *@return
   **/
  bool transAction(void *old_state_data, struct event *event,
                   void *new_state_data);
  /**
   *entryAction
   *@brief
   *进入状态回调
   *@return
   **/
  void entryAction(void *state_data, struct event *event);
  /**
   *exitAction
   *@brief
   *退出状态回调
   *@return
   **/
  void exitAction(void *state_data, struct event *event);
  /**
   *guard
   *@brief
   *状态转移监督函数
   *@return
   **/
  bool guard(const void *condition, struct event *event);
  /**
   *registerTransActionCallback
   *@brief
   *注册状态转移回调函数
   *@return
   **/
  bool registerTransActionCallback(const event *p_event,
                                   std::function<bool()> p_call_back);
  /**
   *registerEntryActionCallback
   *@brief
   *注册状态进入回调函数
   *@return
   **/
  bool registerEntryActionCallback(const void *state_data,
                                   std::function<void()> p_callback);
  /**
   *registerExitActionCallback
   *@brief
   *注册状态退出回调函数
   *@return
   **/
  bool registerExitActionCallback(const void *state_data,
                                  std::function<void()> p_callback);

 private:
  stateMachine slam_state_machine_;

  state unkown_;
  state localization_;
  state init_localization_;
  state mapping_;
  state error_;

  AD_MODE cur_mode_;
  AD_STATU cur_state_;

  event event_stop_localization_;
  event event_start_localization_;
  event event_localization_init_succeed_;
  event event_localization_init_failed_;
  event event_save_map_;
  event event_start_mapping_;
  event event_cancle_mapping_;
  event event_return_mapping_;
  event event_stop_;

  std::map<const state *, std::vector<const event *>>
      map_state_to_events_;  ///< 状态对应的事件集合
  std::map<const event *, state *> map_event_to_next_state_;
  std::map<const state *, std::string> map_state_to_name_;
  std::map<std::string, AD_STATU> map_state_name_to_statu_;
  std::map<AD_STATU, const state *> map_statu_to_state_;
  std::map<const event *, std::function<bool()>> map_event_callback_;
  std::map<const void *, std::function<void()>> map_entry_to_callback_;
  std::map<const void *, std::function<void()>> map_exit_to_callback_;
  std::map<EventName, std::function<bool()>> map_eventname_to_callback_;

  std::map<const event *, std::string> map_event_to_name_;
  std::map<EventName, event *> map_name_to_event_;

  std::map<AD_MODE, std::string> map_mode_to_name_;
  std::map<std::string, AD_MODE> map_state_name_to_mode_;

  std::mutex state_machine_mutex_;
  std::mutex ad_state_mutex_;
  std::mutex ad_mode_mutex_;

 private:
  SlamStateMachine();
  static std::shared_ptr<SlamStateMachine> ptr_state_Machine_;
};
}  // namespace cvte_lidar_slam
#endif