#include "zmq_server.hpp"
#include <glog/logging.h>
#include <stdio.h>

#define BUFF_SIZE 4 * 1024 * 1024

ZmqService::ZmqService(const std::string &ip, unsigned short int port)
    : ip_(ip), port_(port) {
  LOG(INFO) << "ZmqService";
  context_ = zmq_ctx_new();
  if (context_ == NULL) {
    LOG(ERROR) << "zmq_ctx_new fail";
    return;
  }
  socket_ = zmq_socket(context_, ZMQ_REP);
  if (socket_ == NULL) {
    zmq_ctx_destroy(context_);
    LOG(ERROR) << "zmq_socket fail";
    return;
  }

  buffer_ = new char[BUFF_SIZE];
  if (buffer_ == nullptr) {
    LOG(ERROR) << "new error";
    return;
  }
  memset(buffer_, 0, BUFF_SIZE);

  int linger = 0;
  int timeout = 10000;  // ms
                        //   int recv_msg_count = 10;
  int rc = zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
  // zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
  zmq_setsockopt(socket_, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));
  // zmq_setsockopt(socket_, ZMQ_RCVHWM, &recv_msg_count,
  // sizeof(recv_msg_count));

  std::string server_addr = "tcp://" + ip_ + ":" + std::to_string(port_);
  if (zmq_bind(socket_, server_addr.c_str()) < 0) {
    if (socket_ != NULL) {
      zmq_close(socket_);
    }

    if (context_ != NULL) {
      zmq_ctx_destroy(context_);
    }
    LOG(ERROR) << "zmq_bind fail";
    return;
  }
  LOG(INFO) << "bind addr " << server_addr << " success";
}

ZmqService::~ZmqService() {
  LOG(INFO) << "~ZmqService";
  b_exit_ = true;
  if (socket_ != NULL) {
    zmq_close(socket_);
    socket_ = NULL;
  }

  if (context_ != NULL) {
    zmq_ctx_destroy(context_);
    context_ = NULL;
  }

  if (buffer_ != nullptr) {
    delete[] buffer_;
    buffer_ = nullptr;
  }

  if (handle_thread_.joinable()) {
    handle_thread_.join();
  }
}

void ZmqService::setCallback(requestHandleFunc func) {
  std::lock_guard<std::mutex> lock(request_mutex_);
  LOG(INFO) << "set  callback ";
  callback_ = func;
}

void ZmqService::startHandleCommand() {
  handle_thread_ = std::thread([this]() {
    using namespace std::chrono_literals;
    while (!b_exit_) { handleRequest(); }
  });
}

void ZmqService::handleRequest() {
  while (!b_exit_) {
    memset(buffer_, 0, BUFF_SIZE);
    int bytes = zmq_recv(socket_, buffer_, BUFF_SIZE, 0);
    if (bytes < 0) {
      LOG(ERROR) << "recv message faild " << bytes << " " << strerror(errno);
      continue;
    }
    buffer_[bytes] = '\0';
    // LOG(INFO) << "[Server] Recevied Request Message: " << bytes
    //           << "bytes, content: " << buffer_;

    //业务处理 响应
    std::string req = buffer_;

    if (callback_) {
      auto rmw_id = std::make_shared<rmw_request_id_t>();
      auto request = std::make_shared<
          mission_manager_msgs::srv::MissionManager::Request>();
      auto response = std::make_shared<
          mission_manager_msgs::srv::MissionManager::Response>();
      request->send_data = req;

      callback_(rmw_id, request, response);

      int bytes = zmq_send(socket_, response->ack_data.data(),
                           response->ack_data.size(), 0);

      //   LOG(INFO) << "[Server] Sended Reply Message:  " << bytes << " bytes";
    } else {
      LOG(ERROR) << "callback_ is NULL";
    }
  }
}
