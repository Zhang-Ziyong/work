/*
 * @Author: your name
 * @Date: 2020-07-15 11:05:29
 * @LastEditTime: 2020-07-28 16:24:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /cvte_lidar_slam/src/state_machine/slam_state_machine.cpp
 */
#include "slam_state_machine.hpp"
#include "glog/logging.h"
#include <functional>
#include <binders.h>

namespace cvte_lidar_slam {
std::shared_ptr<SlamStateMachine> SlamStateMachine::ptr_state_Machine_ =
    nullptr;

SlamStateMachine::SlamStateMachine() {
  initState();
  stateM_init(&slam_state_machine_, &unkown_, &error_);
  cur_mode_ = AD_MODE::AD_UNKNOW;
  cur_state_ = AD_STATU::UNKNOW;
  LOG(INFO) << "init slam state machine as: " << getCurrentStateName()
            << std::endl;
}

std::shared_ptr<SlamStateMachine> SlamStateMachine::getInstance() {
  if (ptr_state_Machine_ == nullptr) {
    ptr_state_Machine_.reset(new SlamStateMachine());
  }
  return ptr_state_Machine_;
}

void SlamStateMachine::initState() {
  event_stop_localization_ = {
      LIDAR_SLAM_EVENT, new eventPayload("event_stop_localization", "unknow")};
  event_start_localization_ = {
      LIDAR_SLAM_EVENT,
      new eventPayload("event_start_localization", "init_localization")};
  event_localization_init_succeed_ = {
      LIDAR_SLAM_EVENT,
      new eventPayload("event_localization_init_succeed", "localization")};
  // event_localization_init_failed_ = {LIDAR_SLAM_EVENT, new
  // eventPayload("event_localization_init_failed", "unknow")};
  event_save_map_ = {LIDAR_SLAM_EVENT,
                     new eventPayload("event_save_map_", "unknow")};
  event_start_mapping_ = {LIDAR_SLAM_EVENT,
                          new eventPayload("event_start_mapping", "mapping")};
  event_cancle_mapping_ = {LIDAR_SLAM_EVENT,
                           new eventPayload("event_cancle_mapping", "unknow")};
  event_return_mapping_ = {LIDAR_SLAM_EVENT,
                           new eventPayload("event_return_mapping", "mapping")};
  event_stop_ = {LIDAR_SLAM_EVENT, new eventPayload("stop", "unknow")};

  map_name_to_event_ = {
      {STOP_LOCALIZAION, &event_stop_localization_},
      {START_LOCALIZATION, &event_start_localization_},
      {INIT_LOCALIZATION_SUCCESS, &event_localization_init_succeed_},
      {SAVE_MAP, &event_save_map_},
      {START_MAPPING, &event_start_mapping_},
      {CANCLE_MAPPING, &event_cancle_mapping_},
      {RETURN_MAPPING, &event_return_mapping_},
      {STOP, &event_stop_}};

  map_event_to_name_ = {
      {&event_stop_localization_,
       ((eventPayload *) event_stop_localization_.data)->condition_},
      {&event_start_localization_,
       ((eventPayload *) event_start_localization_.data)->condition_},
      {&event_localization_init_succeed_,
       ((eventPayload *) event_localization_init_succeed_.data)->condition_},
      {&event_save_map_, ((eventPayload *) event_save_map_.data)->condition_},
      {&event_start_mapping_,
       ((eventPayload *) event_start_mapping_.data)->condition_},
      {&event_cancle_mapping_,
       ((eventPayload *) event_cancle_mapping_.data)->condition_},
      {&event_return_mapping_,
       ((eventPayload *) event_return_mapping_.data)->condition_},
      {&event_stop_, ((eventPayload *) event_stop_.data)->condition_}};

  map_event_to_next_state_ = {
      {&event_stop_localization_, &unkown_},
      {&event_start_localization_, &init_localization_},
      {&event_localization_init_succeed_, &localization_},
      {&event_save_map_, &unkown_},
      {&event_start_mapping_, &mapping_},
      {&event_cancle_mapping_, &unkown_},
      {&event_return_mapping_, &mapping_},
      {&event_stop_, &unkown_}};

  map_state_to_events_ = {
      {&unkown_, {&event_start_mapping_, &event_start_localization_}},
      {&localization_,
       {&event_stop_localization_, &event_return_mapping_, &event_stop_}},
      {&init_localization_,
       {&event_localization_init_succeed_, &event_stop_localization_,
        &event_stop_}},
      {&mapping_, {&event_cancle_mapping_, &event_save_map_, &event_stop_}}};

  map_state_to_name_ = {{&unkown_, "unknow"},
                        {&init_localization_, "init_localization"},
                        {&localization_, "localization"},
                        {&mapping_, "mapping"}};

  map_state_name_to_statu_ = {
      {"unknow", AD_STATU::UNKNOW},
      {"init_localization", AD_STATU::INIT_LOCALIZATION},
      {"localization", AD_STATU::LOCALIZATION},
      {"mapping", AD_STATU::MAPPING}};

  map_state_name_to_mode_ = {{"unknow", AD_MODE::AD_UNKNOW},
                             {"init_localization", AD_MODE::AD_LOCALIZATION},
                             {"localization", AD_MODE::AD_LOCALIZATION},
                             {"mapping", AD_MODE::AD_MAPPING}};

  map_statu_to_state_ = {{AD_STATU::UNKNOW, &unkown_},
                         {AD_STATU::INIT_LOCALIZATION, &init_localization_},
                         {AD_STATU::LOCALIZATION, &localization_},
                         {AD_STATU::MAPPING, &mapping_}};

  map_mode_to_name_ = {{AD_MODE::AD_LOCALIZATION, "localization"},
                       {AD_MODE::AD_MAPPING, "mapping"},
                       {AD_MODE::AD_UNKNOW, "unknow"}};

  registerState(&unkown_);
  registerState(&localization_);
  registerState(&init_localization_);
  registerState(&mapping_);
}

void SlamStateMachine::registerState(state *p_slam_state,
                                     state *p_entry_state) {
  auto state_to_name_it = map_state_to_name_.find(p_slam_state);
  if (state_to_name_it != map_state_to_name_.end()) {
    p_slam_state->data = (void *) &state_to_name_it->second;
  } else {
    LOG(ERROR) << "register state error, can't find state's name";
  }
  p_slam_state->entryAction =
      std::bind(&SlamStateMachine::entryAction, this, std::placeholders::_1,
                std::placeholders::_2);
  p_slam_state->exitAction =
      std::bind(&SlamStateMachine::exitAction, this, std::placeholders::_1,
                std::placeholders::_2);
  p_slam_state->entryState = p_entry_state;

  auto state_to_event_it = map_state_to_events_.find(p_slam_state);
  if (state_to_event_it != map_state_to_events_.end()) {
    std::vector<const event *> v_support_events = state_to_event_it->second;
    if (!v_support_events.empty()) {
      transition *p_trans_list = new transition[v_support_events.size()];
      for (size_t i = 0; i < v_support_events.size(); i++) {
        const event *p_event = v_support_events[i];
        auto event_to_name_it = map_event_to_name_.find(p_event);
        auto even_to_next_state_it = map_event_to_next_state_.find(p_event);
        if (event_to_name_it != map_event_to_name_.end() &&
            even_to_next_state_it != map_event_to_next_state_.end()) {
          registerTransition(p_trans_list[i], &event_to_name_it->second,
                             even_to_next_state_it->second);
        } else {
          LOG(ERROR) << "can't find event name, or event trans state";
        }
      }
      p_slam_state->transitions = p_trans_list;
      p_slam_state->numTransitions = v_support_events.size();
    } else {
      LOG(ERROR) << "cant find state support event";
    }
  }
}

void SlamStateMachine::registerTransition(transition &trans,
                                          std::string *condition,
                                          state *next_state) {
  trans.eventType = LIDAR_SLAM_EVENT;
  trans.condition = (void *) condition;
  trans.guard = std::bind(&SlamStateMachine::guard, this, std::placeholders::_1,
                          std::placeholders::_2);
  trans.action =
      std::bind(&SlamStateMachine::transAction, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3);
  trans.nextState = next_state;
}

bool SlamStateMachine::sendEvent(const EventName &event_name) {
  auto event_it = map_name_to_event_.find(event_name);
  if (event_it != map_name_to_event_.end()) {
    std::lock_guard<std::mutex> lock(state_machine_mutex_);
    int ret = stateM_handleEvent(&slam_state_machine_, event_it->second);
    if (ret == stateM_stateLoopSelf) {
      LOG(ERROR) << "error: stateM_stateLoopSelf";
      return false;
    } else if (ret == stateM_finalStateReached) {
      LOG(ERROR) << "error: stateM_finalStateReached";
      return false;
    } else if (ret == stateM_stateChanged) {
      LOG(INFO) << "trans state succssfully";
      return true;
    } else {
      LOG(ERROR) << "unkonw error";
      return false;
    }
  } else {
    if (map_eventname_to_callback_.find(event_name) !=
        map_eventname_to_callback_.end()) {
      return map_eventname_to_callback_[event_name]();
    }
    LOG(ERROR) << "can't find event by event_name";
    return false;
  }
}

std::string SlamStateMachine::getCurrentStateName() {
  std::lock_guard<std::mutex> lock(state_machine_mutex_);
  auto state_to_name_it =
      map_state_to_name_.find(slam_state_machine_.currentState);
  if (state_to_name_it != map_state_to_name_.end()) {
    return state_to_name_it->second;
  } else {
    LOG(ERROR) << "unkown state";
    return "error";
  }
}

std::string SlamStateMachine::getCurrentModeName() {
  AD_MODE cur_mode = getCurrentMode();
  if (map_mode_to_name_.find(cur_mode) != map_mode_to_name_.end()) {
    return map_mode_to_name_[cur_mode];
  } else {
    return "error";
  }
}

AD_STATU SlamStateMachine::getCurrentState() {
  return cur_state_;
}

AD_MODE SlamStateMachine::getCurrentMode() {
  return cur_mode_;
}

bool SlamStateMachine::registerTransActionCallback(
    const event *p_event, std::function<bool()> p_call_back) {
  auto iter = map_event_callback_.find(p_event);
  if (iter == map_event_callback_.end()) {
    map_event_callback_[p_event] = p_call_back;
    LOG(ERROR) << "register actiong callback";
    return true;
  } else {
    LOG(ERROR) << "event has trans action callback";
    return false;
  }
}
bool SlamStateMachine::registerTransActionCallback(
    const EventName &event_name, std::function<bool()> p_call_back) {
  auto iter = map_name_to_event_.find(event_name);
  if (iter != map_name_to_event_.end()) {
    return registerTransActionCallback(iter->second, p_call_back);
  } else {
    if (map_eventname_to_callback_.find(event_name) ==
        map_eventname_to_callback_.end()) {
      LOG(ERROR) << "****************register event name**********";
      map_eventname_to_callback_[event_name] = p_call_back;
    }
    return true;
  }
}

bool SlamStateMachine::registerEntryActionCallback(
    const AD_STATU &state_name, std::function<void()> p_callback) {
  auto iter = map_statu_to_state_.find(state_name);
  if (iter != map_statu_to_state_.end()) {
    return registerEntryActionCallback(&map_state_to_name_[iter->second],
                                       p_callback);
  } else {
    LOG(ERROR) << "can't find state by state_name";
    return false;
  }
}

bool SlamStateMachine::registerEntryActionCallback(
    const void *state_data, std::function<void()> p_callback) {
  LOG(INFO) << *(std::string *) state_data << " register entry action callback";
  auto iter = map_entry_to_callback_.find(state_data);
  if (iter == map_entry_to_callback_.end()) {
    map_entry_to_callback_[state_data] = p_callback;
    return true;
  } else {
    LOG(ERROR) << (char *) state_data << " has entry action callback";
    return false;
  }
}

bool SlamStateMachine::registerExitActionCallback(
    const void *state_data, std::function<void()> p_callback) {
  auto iter = map_exit_to_callback_.find(state_data);
  if (iter == map_exit_to_callback_.end()) {
    map_exit_to_callback_[state_data] = p_callback;
    return true;
  } else {
    LOG(ERROR) << "event has exit action call back";
  }
}

bool SlamStateMachine::transAction(void *old_state_data, struct event *event,
                                   void *new_state_data) {
  eventPayload *event_data = (eventPayload *) event->data;
  // if(strcmp((std::string*)new_state_data, &event_data->expected_state_)) {
  if (event_data->expected_state_ != *(std::string *) new_state_data) {
    LOG(ERROR) << "Unexpected state transition from ["
               << (const char *) old_state_data << "] to ["
               << (const char *) new_state_data
               << "]. And cur event's condition is [" << event_data->condition_
               << "] and it's expected state is ["
               << event_data->expected_state_ << "] that is diff from ["
               << (const char *) new_state_data << "]";
    return false;
  }
  auto iter = map_event_callback_.find(event);
  if (iter != map_event_callback_.end()) {
    return iter->second();
  }

  return true;
}

bool SlamStateMachine::guard(const void *condition, struct event *event) {
  eventPayload *event_data = (eventPayload *) event->data;
  // if(strcmp((const char*)condition, event_data->condition_) == 0){
  if (event_data->condition_ == *(std::string *) condition) {
    return true;
  } else {
    return false;
  }
}

void SlamStateMachine::exitAction(void *state_data, struct event *event) {
  std::string state_name = *(std::string *) state_data;
  eventPayload *even_data = (eventPayload *) event->data;
  LOG(INFO) << "exiting: " << state_name
            << ", by event: " << even_data->condition_;

  auto iter = map_exit_to_callback_.find(state_data);
  if (iter != map_exit_to_callback_.end()) {
    iter->second();
  }
}

void SlamStateMachine::entryAction(void *state_data, struct event *event) {
  std::string state_name = *(std::string *) state_data;
  auto mode_it = map_state_name_to_mode_.find(state_name);
  if (mode_it != map_state_name_to_mode_.end()) {
    cur_mode_ = map_state_name_to_mode_[state_name];
    cur_state_ = map_state_name_to_statu_[state_name];
  } else {
    LOG(ERROR) << "error state data: " << state_name << "trans to mode";
  }
  eventPayload *even_data = (eventPayload *) event->data;
  LOG(INFO) << "entering: " << state_name
            << ", by event: " << even_data->condition_;
  auto iter = map_entry_to_callback_.find(state_data);
  if (iter != map_entry_to_callback_.end()) {
    LOG(INFO) << "*********************enter entry callback*******************";
    iter->second();
  } else {
    LOG(WARNING) << state_name << " can't find entry callback";
  }
}

}  // namespace cvte_lidar_slam