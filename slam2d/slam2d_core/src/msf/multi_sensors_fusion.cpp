#include "msf/multi_sensors_fusion.hpp"

#include "glog/logging.h"
#include <iomanip>  // 需要加上头文件
#include <utility>

namespace slam2d_core {
namespace msf {

MultiSensorsFusion::MultiSensorsFusion(
    const MultiSensorsFusionOptions &options) {
  options_ = options;
  status_ = STATUS::UNINITIALIZED;
  initializing_ = false;
  cnt_ = 0;
}

MultiSensorsFusion::~MultiSensorsFusion() {
  clear();
}

void MultiSensorsFusion::clear() {
  // 清空节点
  nodes_.clear();
  predicted_buffer_.clear();
  predicted_constrains_.clear();
  node_id_ = 0;
  new_node_index_ = 0;

  // 清空雷达里程计节点
  laserodom_nodes_.clear();
  laserodom_buffer_.clear();
  laserodom_constrains_.clear();
  laserodom_node_id_ = 0;
  new_laserodom_node_index_ = 0;

  // 清空amcl 约束
  amcl_unary_point_constrain_.reset();
  amcl_unary_constrains_.reset();
  amcl_unary_point_constrain_buffer_.reset();
  amcl_unary_constrain_buffer_.reset();

  initializing_ = false;
  init_predicted_ = false;

  status_ = STATUS::UNINITIALIZED;
}

void MultiSensorsFusion::setInitializeNode(const MSFNode &init_node) {
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  LOG(INFO) << "set msf init-pose: " << init_node.global_pose.x << " "
            << init_node.global_pose.y << " " << init_node.global_pose.yaw;
  cnt_ = 0;
  init_node_ = init_node;
  last_optimize_pose_ = init_node.global_pose;
  last_optimize_time_ = init_node.time;
  initializing_ = true;
  status_ = STATUS::LOCALIZATION_UNSTABLE;
}

bool MultiSensorsFusion::newAmclUnaryConstrain(const double &huber_loss) {
  if (nullptr != amcl_unary_constrains_) {
    LOG(INFO) << "amcl uanry constrain is already exist.";
    return false;
  } else {
    amcl_unary_constrains_ = std::make_shared<std::list<UnaryConstrain>>();
    amcl_unary_constrain_buffer_ =
        std::make_shared<std::list<UnaryConstrainData>>();
    mutex_ = std::make_shared<std::recursive_mutex>();
    huber_loss_ = huber_loss;
    return true;
  }
}

bool MultiSensorsFusion::addPredictedConstrainData(
    const UnaryConstrainData &odom_new_data,
    const UnaryConstrainData &laserodom_new_data) {
  if (!initializing_ || !init_predicted_) {
    last_predicted_data_ = odom_new_data;
    if (options_.use_laserodom_constrain) {
      last_laserodom_data_ = laserodom_new_data;
    }
    init_predicted_ = true;
    return false;
  } else {
    std::lock_guard<std::recursive_mutex> lck(predicted_mutex_);
    predicted_buffer_.push_back(odom_new_data);
    if (options_.use_laserodom_constrain) {
      laserodom_buffer_.push_back(laserodom_new_data);
    }
    // LOG(INFO) << std::fixed << std::setprecision(3) << " odom_new_data stamp
    // "
    //           << common::toUniversal(odom_new_data.time) / 1e7
    //           << " laserodom_new_data stamp "
    //           << common::toUniversal(laserodom_new_data.time) / 1e7;
  }
  return true;
}

bool MultiSensorsFusion::addAmclUnaryConstrainData(
    const UnaryConstrainData &new_data) {
  if (!initializing_) {
    return false;
  } else {
    std::lock_guard<std::recursive_mutex> lck(*mutex_);
    amcl_unary_constrain_buffer_->push_back(new_data);
    return true;
  }
}

bool MultiSensorsFusion::buildUnaryConstrain(
    const std::shared_ptr<std::list<UnaryConstrain>> &constrains,
    const std::shared_ptr<std::list<UnaryConstrainData>> &buffer) {
  UnaryConstrain uc;
  bool find_node;
  // 里程计对应数据
  std::list<MSFNode>::iterator node_iter, last_node_iter;
  Pose2d node_diff, node_pose_bias;
  // 激光里程计对应数据
  // std::list<MSFNode>::iterator laserodom_node_iter, laserodom_last_node_iter;
  // Pose2d laserodom_node_diff, laserodom_node_pose_bias;
  // amcl 应用数据
  double tbegin, tend, node_time, lambda;

  for (std::list<UnaryConstrainData>::iterator buffer_iter = buffer->begin();
       buffer_iter != buffer->end(); buffer_iter++) {
    uc.time = buffer_iter->time;
    uc.global_pose = buffer_iter->global_pose;
    uc.cov = buffer_iter->cov;

    find_node = false;
    node_iter = nodes_.begin();
    // laserodom_node_iter = laserodom_nodes_.begin();
    // LOG(INFO) << " the size of nodes_ " << nodes_.size() << " node_id "
    //           << nodes_.back().node_id << " size of laserodom_nodes_ "
    //           << laserodom_nodes_.size() << " laser node_id "
    //           << laserodom_nodes_.back().node_id;

    // 里程计对应node
    std::advance(node_iter, new_node_index_);
    last_node_iter = node_iter;
    last_node_iter--;

    // 激光里程计对应node
    // std::advance(laserodom_node_iter, new_laserodom_node_index_);
    // laserodom_last_node_iter = laserodom_node_iter;
    // laserodom_last_node_iter--;

    for (; node_iter != nodes_.end(); node_iter++) {
      // if (laserodom_node_iter->time != node_iter->time ||
      //     laserodom_last_node_iter->time != last_node_iter->time) {
      //   LOG(ERROR)
      //       << " the timestamp didnt same between laserodom data and odom
      //       data";
      // }

      if (uc.time >= last_node_iter->time && uc.time <= node_iter->time) {
        tbegin = common::toSeconds(uc.time - last_node_iter->time);
        tend = common::toSeconds(node_iter->time - uc.time);
        if (!options_.do_time_synchronization) {
          if (tbegin < tend) {
            uc.node_id = last_node_iter->node_id;
            last_node_iter->constrain_connected = true;
            last_node_iter->pc_ptr = buffer_iter->pc_ptr;

            // laserodom_last_node_iter->constrain_connected = true;
            // laserodom_last_node_iter->pc_ptr = buffer_iter->pc_ptr;
          } else {
            uc.node_id = node_iter->node_id;
            node_iter->constrain_connected = true;
            node_iter->pc_ptr = buffer_iter->pc_ptr;

            // laserodom_node_iter->constrain_connected = true;
            // laserodom_node_iter->pc_ptr = buffer_iter->pc_ptr;
          }
          find_node = true;
        } else {
          node_time = common::toSeconds(node_iter->time - last_node_iter->time);
          assert(tbegin <= node_time);
          lambda = tbegin / node_time;

          // node_diff = node_iter->global_pose - last_node_iter->global_pose;
          // 里程计node 变化
          node_diff =
              last_node_iter->global_pose.inverse() * node_iter->global_pose;

          node_pose_bias.x = node_diff.x * lambda;
          node_pose_bias.y = node_diff.y * lambda;
          node_pose_bias.yaw = node_diff.yaw * lambda;
          // 激光里程计node 变化
          // laserodom_node_diff =
          //     laserodom_last_node_iter->global_pose.inverse() *
          //     laserodom_node_iter->global_pose;

          // laserodom_node_pose_bias.x = laserodom_node_diff.x * lambda;
          // laserodom_node_pose_bias.y = laserodom_node_diff.y * lambda;
          // laserodom_node_pose_bias.yaw = laserodom_node_diff.yaw * lambda;

          if (tbegin < tend) {
            if (!last_node_iter->constrain_connected) {
              uc.node_id = last_node_iter->node_id;
              // 添加里程计数据
              last_node_iter->time = uc.time;
              last_node_iter->global_pose =
                  last_node_iter->global_pose * node_pose_bias;
              last_node_iter->constrain_connected = true;
              last_node_iter->pc_ptr = buffer_iter->pc_ptr;

              // 添加激光里程计数据
              // laserodom_last_node_iter->time = uc.time;
              // laserodom_last_node_iter->global_pose =
              //     laserodom_last_node_iter->global_pose *
              //     laserodom_node_pose_bias;
              // laserodom_last_node_iter->constrain_connected = true;
              // laserodom_last_node_iter->pc_ptr = buffer_iter->pc_ptr;

              find_node = true;
            }
          } else {
            if (!node_iter->constrain_connected) {
              uc.node_id = node_iter->node_id;
              // 添加里程计数据
              node_iter->time = uc.time;
              node_iter->global_pose =
                  last_node_iter->global_pose * node_pose_bias;
              node_iter->constrain_connected = true;
              node_iter->pc_ptr = buffer_iter->pc_ptr;

              // 添加激光里程计数据
              // laserodom_node_iter->time = uc.time;
              // laserodom_node_iter->global_pose =
              //     laserodom_last_node_iter->global_pose * node_pose_bias;
              // laserodom_node_iter->constrain_connected = true;
              // laserodom_node_iter->pc_ptr = buffer_iter->pc_ptr;

              find_node = true;
            }
          }
        }
        break;
      }

      last_node_iter++;
      // laserodom_node_iter++;
      // laserodom_last_node_iter++;
    }
    if (find_node) {
      constrains->push_back(uc);
    } else {
      // LOG(INFO) << "(buildUnaryConstrain) don't find node :" << uc.time
      //           << " this constrain will be delete.";
    }
  }
  return true;
}

bool MultiSensorsFusion::buildLaserodomNodes(
    const std::shared_ptr<std::list<UnaryConstrainData>> &buffer) {
  UnaryConstrain uc;
  // bool find_node;

  // 里程计对应数据
  // std::list<MSFNode>::iterator node_iter, last_node_iter;
  // Pose2d node_diff, node_pose_bias;
  // 激光里程计对应数据
  std::list<MSFNode>::iterator laserodom_node_iter, laserodom_last_node_iter;
  Pose2d laserodom_node_diff, laserodom_node_pose_bias;
  // amcl 应用数据
  double tbegin, tend, node_time, lambda;

  for (std::list<UnaryConstrainData>::iterator buffer_iter = buffer->begin();
       buffer_iter != buffer->end(); buffer_iter++) {
    uc.time = buffer_iter->time;
    uc.global_pose = buffer_iter->global_pose;
    uc.cov = buffer_iter->cov;

    // find_node = false;
    // node_iter = nodes_.begin();
    laserodom_node_iter = laserodom_nodes_.begin();
    LOG(INFO) << " the size of nodes_ " << nodes_.size() << " node_id "
              << nodes_.back().node_id << " size of laserodom_nodes_ "
              << laserodom_nodes_.size() << " laser node_id "
              << laserodom_nodes_.back().node_id;

    // 里程计对应node
    // std::advance(node_iter, new_node_index_);
    // last_node_iter = node_iter;
    // last_node_iter--;

    // 激光里程计对应node
    std::advance(laserodom_node_iter, new_laserodom_node_index_);
    laserodom_last_node_iter = laserodom_node_iter;
    laserodom_last_node_iter--;

    for (; laserodom_node_iter != laserodom_nodes_.end();
         laserodom_node_iter++) {
      // if (laserodom_node_iter->time != node_iter->time ||
      //     laserodom_last_node_iter->time != last_node_iter->time) {
      //   LOG(ERROR)
      //       << " the timestamp didnt same between laserodom data and odom
      //       data";
      // }

      if (uc.time >= laserodom_last_node_iter->time &&
          uc.time <= laserodom_node_iter->time) {
        tbegin = common::toSeconds(uc.time - laserodom_last_node_iter->time);
        tend = common::toSeconds(laserodom_node_iter->time - uc.time);
        if (!options_.do_time_synchronization) {
          if (tbegin < tend) {
            uc.node_id = laserodom_last_node_iter->node_id;
            // last_node_iter->constrain_connected = true;
            // last_node_iter->pc_ptr = buffer_iter->pc_ptr;

            laserodom_last_node_iter->constrain_connected = true;
            laserodom_last_node_iter->pc_ptr = buffer_iter->pc_ptr;
          } else {
            uc.node_id = laserodom_node_iter->node_id;
            // node_iter->constrain_connected = true;
            // node_iter->pc_ptr = buffer_iter->pc_ptr;

            laserodom_node_iter->constrain_connected = true;
            laserodom_node_iter->pc_ptr = buffer_iter->pc_ptr;
          }
          // find_node = true;
        } else {
          node_time = common::toSeconds(laserodom_node_iter->time -
                                        laserodom_last_node_iter->time);
          assert(tbegin <= node_time);
          lambda = tbegin / node_time;

          // node_diff = node_iter->global_pose - last_node_iter->global_pose;
          // 里程计node 变化
          // node_diff =
          //     last_node_iter->global_pose.inverse() * node_iter->global_pose;

          // node_pose_bias.x = node_diff.x * lambda;
          // node_pose_bias.y = node_diff.y * lambda;
          // node_pose_bias.yaw = node_diff.yaw * lambda;
          // 激光里程计node 变化
          laserodom_node_diff =
              laserodom_last_node_iter->global_pose.inverse() *
              laserodom_node_iter->global_pose;

          laserodom_node_pose_bias.x = laserodom_node_diff.x * lambda;
          laserodom_node_pose_bias.y = laserodom_node_diff.y * lambda;
          laserodom_node_pose_bias.yaw = laserodom_node_diff.yaw * lambda;

          if (tbegin < tend) {
            if (!laserodom_last_node_iter->constrain_connected) {
              uc.node_id = laserodom_last_node_iter->node_id;
              // 添加里程计数据
              // last_node_iter->time = uc.time;
              // last_node_iter->global_pose =
              //     last_node_iter->global_pose * node_pose_bias;
              // last_node_iter->constrain_connected = true;
              // last_node_iter->pc_ptr = buffer_iter->pc_ptr;

              // 添加激光里程计数据
              laserodom_last_node_iter->time = uc.time;
              laserodom_last_node_iter->global_pose =
                  laserodom_last_node_iter->global_pose *
                  laserodom_node_pose_bias;
              laserodom_last_node_iter->constrain_connected = true;
              laserodom_last_node_iter->pc_ptr = buffer_iter->pc_ptr;

              // find_node = true;
            }
          } else {
            if (!laserodom_node_iter->constrain_connected) {
              uc.node_id = laserodom_node_iter->node_id;
              // 添加里程计数据
              // node_iter->time = uc.time;
              // node_iter->global_pose =
              //     last_node_iter->global_pose * node_pose_bias;
              // node_iter->constrain_connected = true;
              // node_iter->pc_ptr = buffer_iter->pc_ptr;

              // 添加激光里程计数据
              laserodom_node_iter->time = uc.time;
              laserodom_node_iter->global_pose =
                  laserodom_last_node_iter->global_pose *
                  laserodom_node_pose_bias;
              laserodom_node_iter->constrain_connected = true;
              laserodom_node_iter->pc_ptr = buffer_iter->pc_ptr;

              // find_node = true;
            }
          }
        }
        break;
      }

      // last_node_iter++;
      // laserodom_node_iter++;
      laserodom_last_node_iter++;
    }
    // if (find_node) {
    //   constrains->push_back(uc);
    // } else {
    //   // LOG(INFO) << "(buildUnaryConstrain) don't find node :" << uc.time
    //   //           << " this constrain will be delete.";
    // }
  }
  return true;
}

Eigen::Matrix3d MultiSensorsFusion::computeBinaryConstrainCov(
    const std::list<MSFNode>::iterator node1,
    const std::list<MSFNode>::iterator node2) {
  Eigen::Matrix3d cov;
  cov.setZero();

  // for test

  Pose2d delta_pose = node1->global_pose.inverse() * node2->global_pose;
  double delta_trans = hypot(delta_pose.x, delta_pose.y);

  cov(0, 0) = cov(1, 1) = std::fabs(delta_trans) * 0.005 * node1->cov(0, 0);
  cov(2, 2) = std::fabs(delta_pose.yaw) * 0.005 * node1->cov(2, 2);

  if (cov(0, 0) < 1e-6) {
    cov(0, 0) = cov(1, 1) = 1e-6;
  }
  if (cov(2, 2) < 1e-6) {
    cov(2, 2) = 1e-6;
  }

  return cov;
}

bool MultiSensorsFusion::build() {
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  if (!initializing_) {
    return false;
  } else if (0 == nodes_.size()) {
    init_node_.node_id = node_id_++;
    nodes_.push_back(init_node_);

    std::lock_guard<std::recursive_mutex> lck(predicted_mutex_);
    last_predicted_data_ = *predicted_buffer_.begin();
    predicted_buffer_.erase(predicted_buffer_.begin());

    if (options_.use_laserodom_constrain) {
      laserodom_node_id_++;
      laserodom_nodes_.push_back(init_node_);  // 添加激光里程计节点

      last_laserodom_data_ = *laserodom_buffer_.begin();
      laserodom_buffer_.erase(laserodom_buffer_.begin());
    }
  }

  // the index of new node data begin
  // 更新nodes_ 获取未进行处理node 的坐标
  new_node_index_ = nodes_.size();
  if (options_.use_laserodom_constrain) {
    new_laserodom_node_index_ = laserodom_nodes_.size();
    if (new_node_index_ != new_laserodom_node_index_) {
      LOG(ERROR) << " nodes size error !!!";
    }
  }
  // TODO: 在只有odom数据插入时不优化
  // build the node
  {
    std::lock_guard<std::recursive_mutex> lck(predicted_mutex_);
    // 里程计位姿变化及其将要添加的新节点
    Pose2d predicted_delta_pose;
    MSFNode new_node;

    // 激光里程计位姿变化及其对应添加的新节点
    Pose2d laserodom_predicted_delta_pose;
    MSFNode laserodom_new_node;

    auto laserodom_predicted_iter = laserodom_buffer_.begin();

    for (auto predicted_iter = predicted_buffer_.begin();
         predicted_iter != predicted_buffer_.end(); predicted_iter++) {
      if (options_.use_laserodom_constrain) {
        if (laserodom_predicted_iter->time != predicted_iter->time ||
            last_laserodom_data_.time != last_predicted_data_.time) {
          LOG(ERROR) << std::fixed << std::setprecision(3)
                     << " the timestamp didnt same between laserodom data and "
                        "odom data"
                     << " laserodom_predicted stamp "
                     << common::toUniversal(laserodom_predicted_iter->time) /
                            1e7
                     << " odom_predicted stamp "
                     << common::toUniversal(predicted_iter->time) / 1e7
                     << " last_laserodom_data stamp "
                     << common::toUniversal(last_laserodom_data_.time) / 1e7
                     << " last_predicted_data stamp "
                     << common::toUniversal(last_predicted_data_.time) / 1e7;
        }
      }

      if (predicted_iter->time > last_predicted_data_.time) {
        // predicted_delta_pose =
        //     predicted_iter->global_pose - last_predicted_data_.global_pose;
        //里程计位姿变化
        predicted_delta_pose = last_predicted_data_.global_pose.inverse() *
                               predicted_iter->global_pose;
        // LOG(INFO) << "predicted_delta_pose : ( " << predicted_delta_pose.x
        //           << " " << predicted_delta_pose.y << " "
        //           << predicted_delta_pose.yaw << ")";
        // 激光里程计位姿变化
        if (options_.use_laserodom_constrain) {
          laserodom_predicted_delta_pose =
              last_laserodom_data_.global_pose.inverse() *
              laserodom_predicted_iter->global_pose;
        }

        bool move_control_restart = false;
        if (std::abs(predicted_iter->global_pose.x) < 0.1 &&
            std::abs(predicted_iter->global_pose.y) < 0.1 &&
            std::abs(predicted_iter->global_pose.yaw) < 0.1) {
          LOG(WARNING) << " ---- move control restart ----";
          move_control_restart = true;
        }  // 处理move_control 重启

        if ((std::abs(predicted_delta_pose.x) > 0.1 ||
             std::abs(predicted_delta_pose.y) > 0.1 ||
             std::abs(predicted_delta_pose.yaw) > 0.5) &&
            move_control_restart) {
          LOG(WARNING) << "dropped data ------";
          last_predicted_data_ = *predicted_iter;
          if (options_.use_laserodom_constrain) {
            last_laserodom_data_ = *laserodom_predicted_iter;
          }
          continue;
        }

        // 里程计数据添加到nodes_ 中
        new_node.node_id = node_id_++;
        new_node.constrain_connected = false;
        new_node.time = predicted_iter->time;
        new_node.global_pose = nodes_.back().global_pose * predicted_delta_pose;
        new_node.cov = predicted_iter->cov;
        nodes_.push_back(new_node);

        if (options_.use_laserodom_constrain) {
          // 激光里程计数据添加到laserodom_nodes_ 中
          if (laserodom_predicted_iter == laserodom_buffer_.end()) {
            LOG(ERROR)
                << " the timestamp didnt same between laserodom data and "
                   "odom data";
          }
          laserodom_new_node.node_id = laserodom_node_id_++;
          laserodom_new_node.constrain_connected = false;
          laserodom_new_node.time = laserodom_predicted_iter->time;
          laserodom_new_node.global_pose = laserodom_nodes_.back().global_pose *
                                           laserodom_predicted_delta_pose;
          laserodom_new_node.cov = laserodom_predicted_iter->cov;
          laserodom_nodes_.push_back(laserodom_new_node);

          //更新上一帧数据
          last_laserodom_data_ = *laserodom_predicted_iter;
          // 移动激光里程计对应的iter
          laserodom_predicted_iter++;
        }

        last_predicted_data_ = *predicted_iter;
      } else {
        laserodom_predicted_iter++;
      }
    }
    // to avoid the problem which get the optimize pose between build and
    // optimze time,and new odom data was come
    last_optimize_pose_ = nodes_.back().global_pose;
    // LOG(WARNING) << "last_optimize_pose_ : ( " << last_optimize_pose_.x <<
    // "
    // "
    //              << last_optimize_pose_.y << " " << last_optimize_pose_.yaw
    //              << ")";

    predicted_buffer_.clear();
    if (options_.use_laserodom_constrain) {
      laserodom_buffer_.clear();
    }
  }

  std::lock_guard<std::recursive_mutex> lck(*mutex_);
  // 上半部分已经添加了node_节点，此时节点里面全部都是里程计绝对值，buildUnaryConstrain()
  // 函数进行里程计数据和amcl 的时间戳匹配
  buildUnaryConstrain(amcl_unary_constrains_, amcl_unary_constrain_buffer_);
  if (options_.use_laserodom_constrain) {
    buildLaserodomNodes(amcl_unary_constrain_buffer_);
  }
  amcl_unary_constrain_buffer_->clear();

  // merge node which had not connect with constrain,and add odom constrain
  if (options_.do_node_merge) {
    // 里程计节点nodes_
    std::list<MSFNode>::iterator node_iter(nodes_.begin());
    std::advance(node_iter, new_node_index_);
    size_t end_node_id = nodes_.back().node_id;

    // 去掉里程计非连接约束节点
    for (; node_iter->node_id < end_node_id && node_iter != nodes_.end();) {
      if (!node_iter->constrain_connected &&
          node_iter->node_id != end_node_id) {
        node_iter = nodes_.erase(node_iter);
      } else {
        node_iter++;
      }
    }

    if (options_.use_laserodom_constrain) {
      // 激光里程计节点laserodom_nodes_
      std::list<MSFNode>::iterator laserodom_node_iter(
          laserodom_nodes_.begin());
      std::advance(laserodom_node_iter, new_laserodom_node_index_);
      size_t laserodom_end_node_id = laserodom_nodes_.back().node_id;
      LOG(INFO) << " end_node_id " << end_node_id << " laserodom_end_node_id "
                << laserodom_end_node_id;

      // 去掉激光里程计非约束节点
      for (; laserodom_node_iter->node_id < laserodom_end_node_id &&
             laserodom_node_iter != laserodom_nodes_.end();) {
        if (!laserodom_node_iter->constrain_connected &&
            laserodom_node_iter->node_id != laserodom_end_node_id) {
          laserodom_node_iter = laserodom_nodes_.erase(laserodom_node_iter);
        } else {
          laserodom_node_iter++;
        }
      }
    }
  }

  // rebuild the predicted constrain (odom)
  // keep the first and last odom constrains whatever it's connected
  // TODO: the odom cov computer
  // 重构并添加里程计约束
  std::list<MSFNode>::iterator node_iter = nodes_.begin();
  std::advance(node_iter, new_node_index_);
  std::list<MSFNode>::iterator last_node_iter = node_iter;
  last_node_iter--;

  for (; node_iter != nodes_.end(); node_iter++) {
    BinaryConstrain bc;
    bc.begin_node_id = last_node_iter->node_id;
    bc.end_node_id = node_iter->node_id;
    bc.begin_time = last_node_iter->time;
    bc.end_time = node_iter->time;
    bc.relative_pose =
        last_node_iter->global_pose.inverse() * node_iter->global_pose;

    // TODO: for test, to avoid the Residuals or Jacobian values evaluating
    // to NaN
    if (std::fabs(bc.relative_pose.x) < 1e-8) {
      bc.relative_pose.x = 1e-8;
      node_iter->global_pose.x += 1e-8;
      // LOG(ERROR) << "predicted data x: < 1e-8";
    }

    if (std::fabs(bc.relative_pose.y) < 1e-8) {
      bc.relative_pose.y = 1e-8;
      node_iter->global_pose.y += 1e-8;
      // LOG(ERROR) << "predicted data y: < 1e-8";
    }
    if (std::fabs(bc.relative_pose.yaw) < 1e-8) {
      bc.relative_pose.yaw = 1e-8;
      node_iter->global_pose.yaw += 1e-8;
      // LOG(ERROR) << "predicted data yaw: < 1e-8";
    }

    bc.cov = computeBinaryConstrainCov(last_node_iter, node_iter);
    predicted_constrains_.push_back(bc);
    last_node_iter = node_iter;
  }

  if (options_.use_laserodom_constrain) {
    // 重构并添加激光里程计约束
    std::list<MSFNode>::iterator laserodom_node_iter = laserodom_nodes_.begin();
    std::advance(laserodom_node_iter, new_laserodom_node_index_);
    std::list<MSFNode>::iterator laserodom_last_node_iter = laserodom_node_iter;
    laserodom_last_node_iter--;

    for (; laserodom_node_iter != laserodom_nodes_.end();
         laserodom_node_iter++) {
      BinaryConstrain bc;
      bc.begin_node_id = laserodom_last_node_iter->node_id;
      bc.end_node_id = laserodom_node_iter->node_id;
      bc.begin_time = laserodom_last_node_iter->time;
      bc.end_time = laserodom_node_iter->time;
      bc.relative_pose = laserodom_last_node_iter->global_pose.inverse() *
                         laserodom_node_iter->global_pose;

      // TODO: for test, to avoid the Residuals or Jacobian values evaluating
      // to NaN
      if (std::fabs(bc.relative_pose.x) < 1e-8) {
        bc.relative_pose.x = 1e-8;
        laserodom_node_iter->global_pose.x += 1e-8;
        // LOG(ERROR) << "predicted data x: < 1e-8";
      }

      if (std::fabs(bc.relative_pose.y) < 1e-8) {
        bc.relative_pose.y = 1e-8;
        laserodom_node_iter->global_pose.y += 1e-8;
        // LOG(ERROR) << "predicted data y: < 1e-8";
      }
      if (std::fabs(bc.relative_pose.yaw) < 1e-8) {
        bc.relative_pose.yaw = 1e-8;
        laserodom_node_iter->global_pose.yaw += 1e-8;
        // LOG(ERROR) << "predicted data yaw: < 1e-8";
      }

      bc.cov = computeBinaryConstrainCov(laserodom_last_node_iter,
                                         laserodom_node_iter);
      laserodom_constrains_.push_back(bc);  // 激光里程计约束
      laserodom_last_node_iter = laserodom_node_iter;
    }
  }

  // 删除里程计节点和约束
  // delete some old node and constrains
  if (nodes_.size() > options_.WINDOW_SIZE) {
    size_t last_reverse_node_id = 0;

    std::list<MSFNode>::iterator node_iter = nodes_.begin();
    std::advance(node_iter, nodes_.size() - options_.WINDOW_SIZE);
    last_reverse_node_id = node_iter->node_id;

    for (auto node_iter = nodes_.begin(); node_iter != nodes_.end();) {
      if (node_iter->node_id <= last_reverse_node_id) {
        node_iter = nodes_.erase(node_iter);
      } else {
        break;
      }
    }  // nodes

    for (auto predicted_iter = predicted_constrains_.begin();
         predicted_iter != predicted_constrains_.end();) {
      if (predicted_iter->begin_node_id <= last_reverse_node_id) {
        predicted_iter = predicted_constrains_.erase(predicted_iter);
      } else {
        break;
      }
    }  // constrains

    for (auto uc_iter = amcl_unary_constrains_->begin();
         uc_iter != amcl_unary_constrains_->end();) {
      if (uc_iter->node_id <= last_reverse_node_id) {
        uc_iter = amcl_unary_constrains_->erase(uc_iter);
      } else {
        break;
      }
    }
  }

  if (options_.use_laserodom_constrain) {
    // 删除激光里程计节点和约束
    // delete some old node and constrains
    if (laserodom_nodes_.size() > options_.WINDOW_SIZE) {
      size_t last_reverse_node_id = 0;

      std::list<MSFNode>::iterator laserodom_last_node_iter =
          laserodom_nodes_.begin();
      std::advance(laserodom_last_node_iter,
                   laserodom_nodes_.size() - options_.WINDOW_SIZE);
      last_reverse_node_id = laserodom_last_node_iter->node_id;

      for (auto laserodom_last_node_iter = laserodom_nodes_.begin();
           laserodom_last_node_iter != laserodom_nodes_.end();) {
        if (laserodom_last_node_iter->node_id <= last_reverse_node_id) {
          laserodom_last_node_iter =
              laserodom_nodes_.erase(laserodom_last_node_iter);
        } else {
          break;
        }
      }  // nodes

      for (auto laserodom_iter = laserodom_constrains_.begin();
           laserodom_iter != laserodom_constrains_.end();) {
        if (laserodom_iter->begin_node_id <= last_reverse_node_id) {
          laserodom_iter = laserodom_constrains_.erase(laserodom_iter);
        } else {
          break;
        }
      }  // constrains
    }
  }

  return true;
}

const std::list<MSFNode>::iterator MultiSensorsFusion::findNode(
    const size_t node_id) {
  std::list<MSFNode>::iterator node_iter = nodes_.begin();
  for (; node_iter != nodes_.end(); node_iter++) {
    if (node_id == node_iter->node_id) {
      return node_iter;
    }
  }
  return node_iter;
}

const std::list<MSFNode>::iterator MultiSensorsFusion::findLaserodomNode(
    const size_t node_id) {
  std::list<MSFNode>::iterator node_iter = laserodom_nodes_.begin();
  for (; node_iter != laserodom_nodes_.end(); node_iter++) {
    if (node_id == node_iter->node_id) {
      return node_iter;
    }
  }
  return node_iter;
}

void MultiSensorsFusion::addBinaryResidualBlock(
    ceres::Problem &problem, ceres::LossFunction *loss_function,
    ceres::LocalParameterization *alp,
    const std::list<BinaryConstrain> &binary_constrains) {
  std::list<MSFNode>::iterator node_begin_iter, node_end_iter;
  Eigen::Matrix3d sqrt_information;
  RelativePoseFactor *cost_function;

  for (auto constrain_iter = binary_constrains.begin();
       constrain_iter != binary_constrains.end(); constrain_iter++) {
    node_begin_iter = findNode(constrain_iter->begin_node_id);
    node_end_iter = findNode(constrain_iter->end_node_id);

    if (nodes_.end() == node_begin_iter || nodes_.end() == node_end_iter) {
      LOG(WARNING) << "add Binary ResidualBlocks error, can not find the node "
                   << constrain_iter->begin_node_id << " "
                   << constrain_iter->end_node_id;
    } else {
      sqrt_information = constrain_iter->cov.inverse().llt().matrixL();
      // ceres::CostFunction *cost_function =
      //     ceres::examples::RelativePoseErrorTerm::Create(
      cost_function = new RelativePoseFactor(
          constrain_iter->relative_pose.x, constrain_iter->relative_pose.y,
          constrain_iter->relative_pose.yaw, sqrt_information);

      problem.AddResidualBlock(
          cost_function, loss_function, &node_begin_iter->global_pose.x,
          &node_begin_iter->global_pose.y, &node_begin_iter->global_pose.yaw,
          &node_end_iter->global_pose.x, &node_end_iter->global_pose.y,
          &node_end_iter->global_pose.yaw);

      problem.SetParameterization(&node_begin_iter->global_pose.yaw, alp);
      problem.SetParameterization(&node_end_iter->global_pose.yaw, alp);
    }
  }
}

void MultiSensorsFusion::addUnaryResidualBlock(
    ceres::Problem &problem, ceres::LossFunction *loss_function,
    ceres::LocalParameterization *alp,
    const std::list<UnaryConstrain> &unary_constrains) {
  std::list<MSFNode>::iterator node_iter;
  Eigen::Matrix3d sqrt_information;
  GlobalPoseFactor *cost_function;
  for (auto constrain_iter = unary_constrains.begin();
       constrain_iter != unary_constrains.end(); constrain_iter++) {
    node_iter = findNode(constrain_iter->node_id);
    if (nodes_.end() == node_iter) {
      LOG(WARNING) << "add Unary ResidualBlocks error, can not find the node "
                   << constrain_iter->node_id << std::endl;
    } else {
      sqrt_information = constrain_iter->cov.inverse().llt().matrixL();

      // ceres::CostFunction *cost_function =
      //     GlobalPoseErrorTerm::Create(
      cost_function = new GlobalPoseFactor(
          constrain_iter->global_pose.x, constrain_iter->global_pose.y,
          constrain_iter->global_pose.yaw, sqrt_information);
      problem.AddResidualBlock(
          cost_function, loss_function, &node_iter->global_pose.x,
          &node_iter->global_pose.y, &node_iter->global_pose.yaw);

      problem.SetParameterization(&node_iter->global_pose.yaw, alp);
    }
  }
}

bool MultiSensorsFusion::optimize() {
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  if (!initializing_) {
    return false;
  }
  ceres::Problem problem;
  ceres::LocalParameterization *angle_local_parameterization =
      AngleLocalParameterization::Create();

  // build odom ceres problem
  {
    ceres::LossFunction *loss_function = NULL;
    addBinaryResidualBlock(problem, loss_function, angle_local_parameterization,
                           predicted_constrains_);
  }  // predicted constrains

  // build laser odom ceres problem
  {
    if (options_.use_laserodom_constrain) {
      ceres::LossFunction *loss_function = NULL;
      addBinaryResidualBlock(problem, loss_function,
                             angle_local_parameterization,
                             laserodom_constrains_);
    }  // laserodom constrains
  }

  ceres::LossFunction *loss_function;

  loss_function = new ceres::HuberLoss(huber_loss_);
  addUnaryResidualBlock(problem, loss_function, angle_local_parameterization,
                        *amcl_unary_constrains_);

  // fix first in optimize
  // LOG(INFO) << __LINE__;
  if (options_.fix_first_node) {
    problem.AddParameterBlock(&nodes_.begin()->global_pose.x, 1);
    problem.AddParameterBlock(&nodes_.begin()->global_pose.y, 1);
    problem.AddParameterBlock(&nodes_.begin()->global_pose.yaw, 1);

    problem.SetParameterBlockConstant(&nodes_.begin()->global_pose.x);
    problem.SetParameterBlockConstant(&nodes_.begin()->global_pose.y);
    problem.SetParameterBlockConstant(&nodes_.begin()->global_pose.yaw);
  }
  // LOG(INFO) << __LINE__;

  ceres::Solver::Options options;
  options.max_num_iterations = 20;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  last_optimize_time_ = nodes_.back().time;
  last_optimize_pose_ = nodes_.back().global_pose;

  // LOG(INFO) << "opt pose: " << last_optimize_pose_.x << " "
  //           << last_optimize_pose_.y << " " << last_optimize_pose_.yaw;
  // LOG(INFO) << "opt node: " << nodes_.begin()->node_id << " - "
  //           << nodes_.back().node_id;

  if (options_.compute_cov) {
    // compute the cov by ceres
    if (summary.num_unsuccessful_steps > 0 ||
        summary.termination_type == ceres::TerminationType::FAILURE) {
      current_pose_cov_[0] = 1;
      current_pose_cov_[1] = 1;
      current_pose_cov_[2] = 1;
      LOG(ERROR) << "optimize error!";
      LOG(ERROR) << summary.FullReport() << std::endl;
      LOG(ERROR) << graphInfo("optimize error graph");
      status_ = STATUS::LOCALIZATION_UNSTABLE;
      return false;
    } else {
      //   LOG(INFO) << "optimize successful...";
      ceres::Covariance::Options cov_options;
      ceres::Covariance covariance(cov_options);

      std::vector<std::pair<const double *, const double *>> covariance_blocks;
      covariance_blocks.reserve(3);
      covariance_blocks.push_back(std::make_pair(&nodes_.back().global_pose.x,
                                                 &nodes_.back().global_pose.x));
      covariance_blocks.push_back(std::make_pair(&nodes_.back().global_pose.y,
                                                 &nodes_.back().global_pose.y));
      covariance_blocks.push_back(std::make_pair(
          &nodes_.back().global_pose.yaw, &nodes_.back().global_pose.yaw));

      bool cov_get = covariance.Compute(covariance_blocks, &problem);
      double covariance_xx = 0.2;
      double covariance_yy = 0.2;
      double covariance_aa = 0.2;
      if (cov_get) {
        covariance.GetCovarianceBlock(&nodes_.back().global_pose.x,
                                      &nodes_.back().global_pose.x,
                                      &covariance_xx);
        covariance.GetCovarianceBlock(&nodes_.back().global_pose.y,
                                      &nodes_.back().global_pose.y,
                                      &covariance_yy);
        covariance.GetCovarianceBlock(&nodes_.back().global_pose.yaw,
                                      &nodes_.back().global_pose.yaw,
                                      &covariance_aa);
      } else {
        LOG(WARNING) << "covariance.Compute error !!!" << std::endl;
      }

      //   LOG(INFO) << "ceres cov: " << covariance_xx << " " << covariance_yy
      //   << " "
      //             << covariance_aa << std::endl;
      current_pose_cov_[0] = covariance_xx;
      current_pose_cov_[1] = covariance_yy;
      current_pose_cov_[2] = covariance_aa;
    }
  }
  status_ = STATUS::WORKING_NORMALLY;
  return true;
}

bool MultiSensorsFusion::getCov(double &cov_xx, double &cov_yy,
                                double &cov_aa) {
  if (!initializing_) {
    return false;
  }
  cov_xx = current_pose_cov_[0];
  cov_yy = current_pose_cov_[1];
  cov_aa = current_pose_cov_[2];
  return true;
}

bool MultiSensorsFusion::getLocalizationResult(
    Pose2d &locatization_result, common::Time &locatization_time) {
  if (!initializing_) {
    return false;
  }
  std::lock_guard<std::recursive_mutex> lck(predicted_mutex_);
  if (predicted_buffer_.size() > 0) {
    // Pose2d delta_pose =
    //     predicted_buffer_.back().global_pose -
    //     last_predicted_data_.global_pose;
    Pose2d delta_pose = last_predicted_data_.global_pose.inverse() *
                        predicted_buffer_.back().global_pose;
    // LOG(WARNING) << " delta_pose x " << delta_pose.x << " y " <<
    // delta_pose.y
    //            << " yaw " << delta_pose.yaw;
    bool move_control_restart = false;
    if (std::abs(predicted_buffer_.back().global_pose.x) < 0.1 &&
        std::abs(predicted_buffer_.back().global_pose.y) < 0.1 &&
        std::abs(predicted_buffer_.back().global_pose.yaw) < 0.1) {
      // LOG(WARNING) << " ---- move control restart ----";
      move_control_restart = true;
    }  // 处理move_control 重启

    if ((delta_pose.x > 1.0 || delta_pose.y > 0.5 || delta_pose.yaw > 0.5) &&
        move_control_restart) {
      LOG(ERROR) << " delta_pose x " << delta_pose.x << " y " << delta_pose.y
                 << " yaw " << delta_pose.yaw;
      locatization_result = last_locatization_result_;
      return true;
    }

    locatization_result = last_optimize_pose_ * delta_pose;
    last_locatization_result_ = locatization_result;
    locatization_time = predicted_buffer_.back().time;
  } else {
    locatization_result = last_optimize_pose_;
    last_locatization_result_ = locatization_result;
    locatization_time = last_optimize_time_;
  }
  return true;
}

bool MultiSensorsFusion::getUnaryNodes(std::vector<MSFNode> &last_unary_nodes) {
  last_unary_nodes.clear();
  if (!initializing_) {
    return false;
  }
  std::list<MSFNode>::iterator node_iter;
  for (node_iter = nodes_.begin(); node_iter != nodes_.end(); node_iter++) {
    if (node_iter->pc_ptr != nullptr && node_iter->node_id >= cnt_) {
      last_unary_nodes.push_back(*node_iter);
    }
  }
  if (last_unary_nodes.size() > 0) {
    cnt_ = last_unary_nodes.back().node_id + 1;
  }
  return true;
}

std::string MultiSensorsFusion::graphInfo(std::string info_name) {
  std::lock_guard<std::recursive_mutex> lck(predicted_mutex_);
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  std::string info =
      "---------------------- " + info_name + " ----------------------\n";
  info += "msf graph: " + std::to_string(nodes_.begin()->node_id) + " - " +
          std::to_string(nodes_.back().node_id) + '\n';
  info += "node: " + std::to_string(nodes_.size()) + "\n";
  info += "constrains :\n  predicted: " +
          std::to_string(predicted_constrains_.size()) + "\n";

  info += "   amcl_unary_constrains :" +
          std::to_string(amcl_unary_constrains_->size()) + "\n";

  std::string connect;
  std::string pc_ptr;
  for (auto node_iter = nodes_.begin(); node_iter != nodes_.end();
       node_iter++) {
    connect = node_iter->constrain_connected ? " c " : " n ";
    pc_ptr = (node_iter->pc_ptr == nullptr) ? " nullptr " : " realptr ";
    info += std::to_string(node_iter->node_id) + connect + pc_ptr + " (" +
            std::to_string(node_iter->global_pose.x) + "," +
            std::to_string(node_iter->global_pose.y) + "," +
            std::to_string(node_iter->global_pose.yaw) + ")\n";

    for (auto predicted_iter = predicted_constrains_.begin();
         predicted_iter != predicted_constrains_.end(); predicted_iter++) {
      if (predicted_iter->end_node_id == node_iter->node_id) {
        info += "e_predicted (" +
                std::to_string(predicted_iter->relative_pose.x) + "," +
                std::to_string(predicted_iter->relative_pose.y) + "," +
                std::to_string(predicted_iter->relative_pose.yaw) + ")\n";
      }
      if (predicted_iter->begin_node_id == node_iter->node_id) {
        info += "b_predicted\n";
      }
    }

    for (auto uc_iter = amcl_unary_constrains_->begin();
         uc_iter != amcl_unary_constrains_->end(); uc_iter++) {
      if (uc_iter->node_id == node_iter->node_id) {
        info += "amcl_unary (" + std::to_string(uc_iter->global_pose.x) + "," +
                std::to_string(uc_iter->global_pose.y) + "," +
                std::to_string(uc_iter->global_pose.yaw) + ")\n";
      }
    }
  }
  info += "----------------------------------------------------------\n";
  return info;
}

bool MultiSensorsFusion::initialization() {
  return initializing_;
}

size_t MultiSensorsFusion::getPredictedSize() {
  std::lock_guard<std::recursive_mutex> lck(predicted_mutex_);
  return predicted_buffer_.size();
}

}  // namespace msf
}  // namespace slam2d_core
