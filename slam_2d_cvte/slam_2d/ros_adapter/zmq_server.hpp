#pragma once

#include "mission_manager_msgs/srv/mission_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <zmq.h>
#include <string>
#include <thread>
#include <functional>
#include <map>
#include <mutex>
#include <jsoncpp/json/json.h>

using requestHandleFunc = std::function<void(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
        request,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
        response)>;

using requestHandleMap = std::map<std::string, requestHandleFunc>;

class ZmqService {
 public:
  ZmqService(const std::string &ip, unsigned short int port);
  ~ZmqService();
  void setCallback(requestHandleFunc func);
  void startHandleCommand();

 private:
  void handleRequest();

 private:
  requestHandleFunc callback_;
  std::string ip_;
  unsigned short int port_;
  void *context_ = NULL;
  void *socket_ = NULL;
  std::mutex request_mutex_;
  requestHandleMap request_handle_map_;
  bool b_exit_ = false;
  std::thread handle_thread_;

  char *buffer_ = nullptr;
};