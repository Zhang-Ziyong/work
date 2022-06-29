
#include "msf/multi_sensors_fusion.hpp"

#include <glog/logging.h>

#include "msf/pose_graph_error_term.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/config/system_config.hpp"
#include "common/debug_tools/tic_toc.h"
#include "map/map_manager.hpp"

namespace cvte_lidar_slam {
MutliSensorsFusion::MutliSensorsFusion(const MsfConfig &config) {
  config_ = config;
  ptr_l_node_ = std::make_shared<std::list<Node>>();
  ptr_state_machine_ = SlamStateMachine::getInstance();
  ptr_l_ptr_keyframe_buffer_ =
      std::make_shared<std::list<std::shared_ptr<KeyFrame>>>();
  ptr_l_loop_constrain_ = std::make_shared<std::list<LoopConstrain>>();
  ptr_map_manager_ = MapManager::getInstance();

  is_optimize_runing_ = false;
  new_node_size_ = 0;
  new_loop_count_ = 0;
  loop_count_ = 0;
  new_gps_count_ = 0;
  set_gps_ex_ = false;

  T_ex_ = Mathbox::Identity34();
}

MutliSensorsFusion::~MutliSensorsFusion() {}

void MutliSensorsFusion::Reset() {
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  std::lock_guard<std::recursive_mutex> lck_l(loop_constrain_mutex_);
  std::lock_guard<std::recursive_mutex> lck_n(buffer_mutex_);

  ptr_l_node_->clear();
  ptr_l_ptr_keyframe_buffer_->clear();
  ptr_l_loop_constrain_->clear();
  is_optimize_runing_ = false;
  new_node_size_ = 0;
  new_loop_count_ = 0;
  loop_count_ = 0;
  new_gps_count_ = 0;
  set_gps_ex_ = false;

  T_ex_ = Mathbox::Identity34();
}

Node &MutliSensorsFusion::buildMapNode(
    const std::shared_ptr<KeyFrame> ptr_keyframe) {
  Node node;
  auto last_node = ptr_l_node_->back();
  node.ptr_keyframe = ptr_keyframe;
  node.idx = node.ptr_keyframe->index_;

  node.relative_pose_constraint = Mathbox::deltaPose34d(
      last_node.ptr_keyframe->T_wb_, node.ptr_keyframe->T_wb_);
  node.relative_pose_cov = node.ptr_keyframe->map_cov_;
  if (config_.using_gps && node.ptr_keyframe->has_gps_) {
    new_gps_count_++;
    node.gps_valued = true;
    node.gps_pose = node.ptr_keyframe->gps_pos_;
    node.gps_cov = Mat3d::Identity() * node.ptr_keyframe->gps_cov_;
  } else {
    node.gps_valued = false;
  }
  LOG(INFO) << "Map node: " << node.idx;
  ptr_l_node_->push_back(node);
  new_node_size_++;

  return node;
}

Node &MutliSensorsFusion::buildNode(
    const std::shared_ptr<KeyFrame> ptr_keyframe) {
  //   std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  static double relative_cov = config_.relative_cov;
  Node node;
  auto last_node = ptr_l_node_->back();
  node.ptr_keyframe = ptr_keyframe;
  node.idx = node.ptr_keyframe->index_;

  // 定位模式：进行里程计增量和地图匹配结果之间的滑动窗口优化，使输出位姿更平滑；
  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    // 计算相对位姿约束，也就是位姿增量约束，是通过里程计得到的；
    node.relative_pose_constraint =
        Mathbox::deltaPose34d(last_node.ptr_keyframe->getWheelPose(),
                              node.ptr_keyframe->getWheelPose());
    if (std::fabs(node.relative_pose_constraint(0, 3)) < 1e-8) {
      node.relative_pose_constraint(0, 3) = 1e-8;
    }
    if (std::fabs(node.relative_pose_constraint(1, 3)) < 1e-8) {
      node.relative_pose_constraint(1, 3) = 1e-8;
    }
    if (std::fabs(node.relative_pose_constraint(2, 3)) < 1e-8) {
      node.relative_pose_constraint(2, 3) = 1e-8;
    }

    // 计算增量约束协方差；
    node.relative_pose_cov = Mat6d::Identity() * 0.1;  // TODO: 修改协方差
    const double &delta_dis =
        node.relative_pose_constraint.block<3, 1>(0, 3).norm();
    // 如果发生轮子打滑，需要加大里程计约束的协方差；
    if (node.ptr_keyframe->is_wheel_skidded_ || delta_dis > 1.) {
      LOG(INFO) << "Zoom in cov for wheel skidding ";
      node.is_odom_valid = false;
      node.relative_pose_cov *= 100000.0;
    }

    // 利用里程计增量对机器人位姿做预测；
    node.ptr_keyframe->T_wb_ = Mathbox::multiplePose34d(
        last_node.ptr_keyframe->T_wb_, node.relative_pose_constraint);

    // 和地图匹配的结果做为全局位姿约束；
    node.global_pose_constraint = node.ptr_keyframe->T_wm_;
    node.global_pose_cov = node.ptr_keyframe->map_cov_;
  }
  // 建图模式：根据增量式激光匹配结果与闭环结果，进行闭环优化；
  else {
    // 只构建关键帧间的相对位姿约束；
    node.relative_pose_constraint = Mathbox::deltaPose34d(
        last_node.ptr_keyframe->T_wb_, node.ptr_keyframe->T_wb_);
    // node.relative_pose_cov = node.ptr_keyframe->map_cov_;
    // node.relative_pose_cov = Mat6d::Identity() * 0.02;  // TODO: 修改协方差
    node.relative_pose_cov =
        Mat6d::Identity() * relative_cov;  // TODO: 修改协方差

    // LOG(INFO) << "\n" << node.idx << " "
    //           << node.relative_pose_constraint << "\ncov:\n"
    //           << node.relative_pose_cov;
  }

  // 如果有GPS的话，加入GPS全局位置约束；
  if (config_.using_gps && node.ptr_keyframe->has_gps_) {
    new_gps_count_++;
    node.gps_valued = true;
    node.gps_pose = node.ptr_keyframe->gps_pos_;
    node.gps_cov = Mat3d::Identity() * node.ptr_keyframe->gps_cov_;
  } else {
    node.gps_valued = false;
  }
  LOG(INFO) << "node: " << node.idx;

  // if (node.gps_valued) {
  //   LOG(INFO) << "gps:\n" << node.gps_pose << "\ncov:\n" << node.gps_cov;
  // }
  ptr_l_node_->push_back(node);
  new_node_size_++;

  return node;
}

bool MutliSensorsFusion::saveGraph(const std::string &filename) {
  if (ptr_l_loop_constrain_->empty()) {
    LOG(ERROR) << "Empty loop_constrain container";
    return false;
  }
  const std::string &info_file_name = filename + "loop_info.txt";
  std::ofstream loop_info_file;
  loop_info_file.open(info_file_name, std::ios::out | std::ios::trunc);
  loop_info_file << std::fixed;
  if (!loop_info_file.is_open()) {
    LOG(ERROR) << " Can not open file";
    return false;
  }
  loop_info_file << "#format: frame_id1 frame_id2 tx ty tz qw qx qy qz pose_cov"
                 << std::endl;

  for (auto loop_constrain_iter = ptr_l_loop_constrain_->begin();
       loop_constrain_iter != ptr_l_loop_constrain_->end();
       loop_constrain_iter++) {
    auto frame1_id = loop_constrain_iter->loop_node_id1;
    auto frame2_id = loop_constrain_iter->loop_node_id2;
    const Mat34d &relative_pose = loop_constrain_iter->relative_pose;
    const double &pose_cov = loop_constrain_iter->rp_cov(0, 0);
    Eigen::Quaterniond quat(relative_pose.block<3, 3>(0, 0));
    quat.normalize();
    loop_info_file << std::setprecision(0) << frame1_id << ' ' << frame2_id
                   << ' ' << std::setprecision(6) << relative_pose(0, 3) << ' '
                   << relative_pose(1, 3) << ' ' << relative_pose(2, 3) << ' '
                   << quat.x() << ' ' << quat.y() << ' ' << quat.z() << ' '
                   << quat.w() << ' ' << pose_cov << std::endl;
  }
  return true;
}

bool MutliSensorsFusion::loadGraph(const std::string &filename) {
  const std::string &info_file_name = filename + "loop_info.txt";
  std::ifstream loop_info_file;
  loop_info_file.open(info_file_name);
  if (!loop_info_file.is_open()) {
    LOG(ERROR) << "loadGraph: Can not open file";
    return false;
  }
  while (!loop_info_file.eof()) {
    LoopConstrain loop_constrain;
    double pose[8] = {0.};
    std::string s;
    std::getline(loop_info_file, s);
    if (s.front() == '#' || s.empty()) {
      continue;
    }
    std::stringstream ss;
    ss << s;
    ss >> loop_constrain.loop_node_id1;
    ss >> loop_constrain.loop_node_id2;

    for (uint i = 0; i < 8; ++i) { ss >> pose[i]; }
    Eigen::Quaterniond Quat(pose[6], pose[3], pose[4], pose[5]);
    Eigen::Map<Eigen::Vector3d> Trans(pose);
    Quat.normalize();
    loop_constrain.relative_pose.block<3, 3>(0, 0) = Quat.toRotationMatrix();
    loop_constrain.relative_pose.block<3, 1>(0, 3) = Trans;
    loop_constrain.rp_cov = pose[7] * Mat6d::Identity();
    loop_constrain.ptr_keyframe1 =
        ptr_map_manager_->getKeyFrameFromID(loop_constrain.loop_node_id1);
    loop_constrain.ptr_keyframe2 =
        ptr_map_manager_->getKeyFrameFromID(loop_constrain.loop_node_id2);
    // std::cout << loop_constrain.loop_node_id1 << ' ' <<
    // loop_constrain.loop_node_id2 << ' '
    //           << pose[0] << ' ' << pose[1] << ' ' << pose[2] << ' ' <<
    //           pose[3] << ' '
    //           << pose[4] << ' ' << pose[5] << ' ' << pose[6] << ' ' <<
    //           pose[7] << std::endl;
    // std::cout << loop_constrain.relative_pose << std::endl;
    addLoop(loop_constrain);
  }

  new_loop_count_ = 0;
  return true;
}

void MutliSensorsFusion::addMapKeyFrame(
    const std::shared_ptr<KeyFrame> ptr_keyframe) {
  if (AD_MODE::AD_UNKNOW == ptr_state_machine_->getCurrentMode()) {
    LOG(ERROR) << "SYSTEM MODE IS UNKNOWN" << std::endl;
    return;
  }
  if (is_optimize_runing_ && ptr_l_node_->size() > 0) {
    std::lock_guard<std::recursive_mutex> lck_n(buffer_mutex_);
    ptr_l_ptr_keyframe_buffer_->push_back(ptr_keyframe);
  } else {
    std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
    if (ptr_l_ptr_keyframe_buffer_->size() > 0) {
      std::lock_guard<std::recursive_mutex> lck_n(buffer_mutex_);

      for (auto buffer_iter = ptr_l_ptr_keyframe_buffer_->begin();
           buffer_iter != ptr_l_ptr_keyframe_buffer_->end(); buffer_iter++) {
        buildMapNode(*buffer_iter);
      }
      ptr_l_ptr_keyframe_buffer_->clear();
    }
    if (ptr_l_node_->size() == 0) {
      Node node;
      node.ptr_keyframe = ptr_keyframe;
      node.idx = ptr_keyframe->index_;
      if (ptr_keyframe->has_gps_) {
        node.gps_valued = true;
        node.gps_pose = ptr_keyframe->gps_pos_;
        node.gps_cov = Mat3d::Identity() * node.ptr_keyframe->gps_cov_;
      } else {
        node.gps_valued = false;
      }
      ptr_l_node_->push_back(node);
      new_node_size_++;
    } else {
      buildMapNode(ptr_keyframe);
    }
  }
}

void MutliSensorsFusion::addKeyFrame(
    const std::shared_ptr<KeyFrame> ptr_keyframe) {
  if (AD_MODE::AD_UNKNOW == ptr_state_machine_->getCurrentMode()) {
    LOG(ERROR) << "SYSTEM MODE IS UNKNOWN" << std::endl;
    return;
  }
  // 如果优化线程正在忙，则将新添加的关键帧先缓存下来，等到优化结束后再一起优化；
  if (is_optimize_runing_ && ptr_l_node_->size() > 0) {
    std::lock_guard<std::recursive_mutex> lck_n(buffer_mutex_);
    ptr_l_ptr_keyframe_buffer_->push_back(ptr_keyframe);
  } else {
    // 如果空闲，则先遍历缓存的关键帧，构建成节点；
    std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
    if (ptr_l_ptr_keyframe_buffer_->size() > 0) {
      std::lock_guard<std::recursive_mutex> lck_n(buffer_mutex_);

      for (auto buffer_iter = ptr_l_ptr_keyframe_buffer_->begin();
           buffer_iter != ptr_l_ptr_keyframe_buffer_->end(); buffer_iter++) {
        //
        buildNode(*buffer_iter);
      }
      ptr_l_ptr_keyframe_buffer_->clear();
    }

    if (ptr_l_node_->size() == 0) {
      Node node;
      node.ptr_keyframe = ptr_keyframe;
      node.idx = ptr_keyframe->index_;
      if (ptr_keyframe->has_gps_) {
        node.gps_valued = true;
        node.gps_pose = ptr_keyframe->gps_pos_;
        node.gps_cov = Mat3d::Identity() * node.ptr_keyframe->gps_cov_;
      } else {
        node.gps_valued = false;
      }
      ptr_l_node_->push_back(node);
      new_node_size_++;
    } else {
      // 然后将当前关键帧构建为优化节点；
      buildNode(ptr_keyframe);
    }
  }
}

bool MutliSensorsFusion::addLoop(const LoopConstrain &loop) {
  std::lock_guard<std::recursive_mutex> lck_g(loop_constrain_mutex_);
  if (is_optimize_runing_) {
    LOG(WARNING) << "add loop failed, because msf running.";
    return false;
  }

  ptr_l_loop_constrain_->push_back(loop);
  new_loop_count_++;

  LOG(WARNING) << "add loop constrain: " << loop.loop_node_id1 << " and "
               << loop.loop_node_id2;
  LOG(WARNING) << "relative_pose:\n" << loop.relative_pose;
  // LOG(WARNING) << "cov: " << loop.rp_cov;

  return true;
}

void MutliSensorsFusion::setGpsTrans(const Mat34d &gps_trans) {
  T_ex_ = gps_trans;
  set_gps_ex_ = true;
  LOG(INFO) << "set GPSExtrincs\n" << T_ex_;
}

Mat34d MutliSensorsFusion::getGpsTrans() {
  return T_ex_;
}

bool MutliSensorsFusion::optimization() {
  TicToc opt_cost;
  is_optimize_runing_ = true;

  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    if (ptr_l_node_->size() < config_.opt_windows_size / 2 ||
        new_node_size_ < config_.new_node_size_to_opt) {
      is_optimize_runing_ = false;
      return false;
    } else {
      new_node_size_ = 0;
    }
  } else if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    if (new_loop_count_ < config_.opt_loop_count) {
      if (set_gps_ex_ && new_gps_count_ >= config_.gps_count_to_opt) {
        new_gps_count_ = 0;
      } else {
        is_optimize_runing_ = false;
        return false;
      }
    } else {
      new_loop_count_ = 0;
      loop_count_++;
    }
  } else {
    LOG(ERROR) << "opt model error!";
    is_optimize_runing_ = false;
    return false;
  }

  LOG(INFO) << "start optimization";

  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode() &&
      ptr_l_node_->size() > config_.opt_windows_size &&
      config_.opt_windows_size > 0) {
    auto node_iter = ptr_l_node_->begin();
    // 将某个迭代器前进到指定的位置上；
    std::advance(node_iter, ptr_l_node_->size() - config_.opt_windows_size);
    size_t last_reverse_node_id = node_iter->idx;

    // 将超过滑动窗口尺寸的节点从节点链表中删除掉；
    for (auto erase_node_iter = ptr_l_node_->begin();
         erase_node_iter != ptr_l_node_->end();) {
      if (erase_node_iter->idx <= last_reverse_node_id) {
        erase_node_iter = ptr_l_node_->erase(erase_node_iter);
      } else {
        break;
      }
    }

    // 将相关连的闭环约束也删除掉；
    for (auto erase_loop_iter = ptr_l_loop_constrain_->begin();
         erase_loop_iter != ptr_l_loop_constrain_->end(); erase_loop_iter++) {
      if (erase_loop_iter->loop_node_id1 <= last_reverse_node_id ||
          erase_loop_iter->loop_node_id2 <= last_reverse_node_id) {
        erase_loop_iter = ptr_l_loop_constrain_->erase(erase_loop_iter);
      }
    }
  }

  LOG(INFO) << "optimization: " << ptr_l_node_->begin()->idx << " - "
            << ptr_l_node_->back().idx;

  ceres::Problem problem;
  ceres::LocalParameterization *local_parameterization = new PosePara();
  ceres::LossFunction *laser_odom_loss_function = nullptr;
  ceres::LossFunction *wheel_odom_loss_function = nullptr;
  ceres::LossFunction *loop_constrain_loss_function = new ceres::HuberLoss(0.1);
  // ceres::LossFunction *loop_constrain_loss_function = nullptr;
  ceres::LossFunction *map_match_loss_function = new ceres::HuberLoss(0.1);
  ceres::LossFunction *gps_loss_function = new ceres::HuberLoss(0.01);
  // ceres::LossFunction *gps_loss_function = nullptr;

  auto node_iter = ptr_l_node_->begin();
  auto last_node_iter = node_iter;
  node_iter++;
  bool gps_in_graph = false;

  problem.AddParameterBlock(last_node_iter->ptr_keyframe->T_wb_.data(), 12,
                            local_parameterization);

  for (; node_iter != ptr_l_node_->end();
       last_node_iter = node_iter, node_iter++) {
    problem.AddParameterBlock(node_iter->ptr_keyframe->T_wb_.data(), 12,
                              local_parameterization);

    // add map pose constrain
    if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
      if (node_iter->global_pose_cov(0, 0) < 0.5) {
        // Eigen的LLT分解实现了Cholesky分解；
        // 把一个对称正定的矩阵表示成一个下三角矩阵L和其转置的乘积；
        Mat6d sqrt_info =
            Eigen::LLT<Mat6d>(node_iter->global_pose_cov.inverse())
                .matrixL()
                .transpose();

        MapFactor *map_pose_cost_function =
            new MapFactor(node_iter->global_pose_constraint, sqrt_info);

        problem.AddResidualBlock(map_pose_cost_function,
                                 map_match_loss_function,
                                 node_iter->ptr_keyframe->T_wb_.data());
      }
    }

    // add wheel odom constrain
    if (node_iter->is_odom_valid) {
      Mat6d sqrt_info =
          Eigen::LLT<Mat6d>(node_iter->relative_pose_cov.inverse())
              .matrixL()
              .transpose();
      OdomRelativeFactor *wheel_odom_cost_function = new OdomRelativeFactor(
          node_iter->relative_pose_constraint, sqrt_info);

      // LOG(WARNING) << "msf::OdomRelativeFactor::sqrt_info: ";
      // LOG(WARNING) << sqrt_info;

      problem.AddResidualBlock(wheel_odom_cost_function,
                               wheel_odom_loss_function,
                               last_node_iter->ptr_keyframe->T_wb_.data(),
                               node_iter->ptr_keyframe->T_wb_.data());
    }

    // add gps constrain
    if (config_.using_gps && node_iter->gps_valued && set_gps_ex_) {
      Mat3d gps_info = node_iter->gps_cov;
      gps_info(0, 0) = 0.01 / sqrt(gps_info(0, 0));
      gps_info(1, 1) = 0.01 / sqrt(gps_info(1, 1));
      gps_info(2, 2) = 0.01 / sqrt(gps_info(2, 2));
      // GPS2dFactor* gps_cost_function =
      //     new GPS2dFactor(node_iter->gps_pose, gps_info);
      // problem.AddResidualBlock(gps_cost_function, gps_loss_function,
      //                          node_iter->ptr_keyframe->T_wb_.data());

      GPSFactor *gps_with_ex_cost_funtion =
          new GPSFactor(node_iter->gps_pose, gps_info);
      problem.AddResidualBlock(gps_with_ex_cost_funtion, gps_loss_function,
                               node_iter->ptr_keyframe->T_wb_.data(),
                               T_ex_.data());
      gps_in_graph = true;
      // LOG(INFO) << "add gps to opt: " << node_iter->ptr_keyframe->index_;
      // LOG(INFO) << "gps_pose: \n" << node_iter->gps_pose;
      // LOG(INFO) << "gps_map_pose: \n"
      //           << T_ex_.block<3, 3>(0, 0) * node_iter->gps_pose -
      //                  T_ex_.block<3, 1>(0, 3);
      // LOG(INFO) << "lidar_pose:\n" << node_iter->ptr_keyframe->T_wb_;
    }
  }

  int loop_count = 0;
  for (auto loop_constrain_iter = ptr_l_loop_constrain_->begin();
       loop_constrain_iter != ptr_l_loop_constrain_->end();
       loop_constrain_iter++) {
    loop_count++;
    // if (loop_count < ptr_l_loop_constrain_->size())
    // {
    //   continue;
    // }
    auto loop_frame1 = loop_constrain_iter->ptr_keyframe1;
    auto loop_frame2 = loop_constrain_iter->ptr_keyframe2;

    if (nullptr != loop_frame1 && nullptr != loop_frame2) {
      Mat6d sqrt_info = Eigen::LLT<Mat6d>(loop_constrain_iter->rp_cov.inverse())
                            .matrixL()
                            .transpose();
      LoopRelativeFactor *laser_odom_cost_function =
          new LoopRelativeFactor(loop_constrain_iter->relative_pose, sqrt_info);

      // LOG(WARNING) << "msf::LoopFactor::sqrt_info: ";
      // LOG(WARNING) << sqrt_info;

      problem.AddResidualBlock(
          laser_odom_cost_function, loop_constrain_loss_function,
          loop_frame1->T_wb_.data(), loop_frame2->T_wb_.data());
      LOG(WARNING) << "--msf: loop_count: " << loop_count << ", "
                   << loop_frame1->index_ << " and " << loop_frame2->index_;
    }
  }

  if (config_.fix_first_node && !ptr_l_node_->empty()) {
    problem.SetParameterBlockConstant(
        ptr_l_node_->begin()->ptr_keyframe->T_wb_.data());
  }

  if ((AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode() ||
       config_.fix_gps_extrincs) &&
      gps_in_graph) {
    problem.SetParameterBlockConstant(T_ex_.data());
  }

  for (auto node_iter = ptr_l_node_->begin(); node_iter != ptr_l_node_->end();
       node_iter++) {
    problem.SetParameterization(node_iter->ptr_keyframe->T_wb_.data(),
                                local_parameterization);  //
  }

  ceres::Solver::Options options;
  // options.minimizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 10;
  options.num_threads = 2;
  // options.function_tolerance = 1e-6;
  // options.parameter_tolerance = 1e-8;
  // options.max_linear_solver_iterations = 5000;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  LOG(INFO) << "----------------------  msf ----------------------------------";
  LOG(INFO) << summary.FullReport() << std::endl;
  LOG(INFO) << "opt: " << ptr_l_node_->back().ptr_keyframe->T_wb_ << std::endl;
  LOG(INFO) << "node: " << ptr_l_node_->size()
            << "    loop: " << ptr_l_loop_constrain_->size() << std::endl;
  LOG(INFO) << "--------------------------------------------------------------";
  LOG(INFO) << "*********************opt time :" << opt_cost.toc()
            << "*******************";
  // ptr_l_node_->back().ptr_keyframe->T_wm_ =
  // ptr_l_node_->back().ptr_keyframe->T_wb_;

  // TODO: 将loop_count>5 改为优化前后位置变化小于一定阈值？
  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode() &&
      config_.using_gps && !set_gps_ex_ &&
      // ptr_gps_adapter_ != nullptr &&
      loop_count_ > 5) {
    size_t gps_count = 0;
    for (auto nore_iter = ptr_l_node_->begin(); nore_iter != ptr_l_node_->end();
         nore_iter++) {
      if (nore_iter->gps_valued) {
        gps_count++;
      }
    }

    LOG(INFO) << "gps_count: " << gps_count << std::endl;
    if (gps_count >= config_.gps_calculate_tr_count) {
      ceres::Problem gps_problem;
      for (auto node_iter = ptr_l_node_->begin();
           node_iter != ptr_l_node_->end(); node_iter++) {
        if (node_iter->gps_valued) {
          Mat3d cov = node_iter->gps_cov;
          Vec3d gps_pose = node_iter->ptr_keyframe->gps_pos_;
          Vec3d lidar_pose = node_iter->ptr_keyframe->T_wb_.block<3, 1>(0, 3);
          Mat3d sqrt_info = Eigen::LLT<Mat3d>(cov).matrixL().transpose();
          ceres::CostFunction *cost_function =
              new GPSExtrincsFactor(gps_pose, lidar_pose, sqrt_info);
          gps_problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1),
                                       T_ex_.data());
        }
      }

      gps_problem.SetParameterization(T_ex_.data(), new PosePara());
      ceres::Solver::Options options;
      options.minimizer_type = ceres::TRUST_REGION;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &gps_problem, &summary);
      LOG(ERROR) << "calcu gps temp transform";
      LOG(ERROR) << summary.FullReport();
      LOG(ERROR) << "trans:" << T_ex_;
    }
  }

  is_optimize_runing_ = false;

  return true;
}

std::shared_ptr<std::list<Node>> MutliSensorsFusion::getNode() {
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  return ptr_l_node_;
}
std::shared_ptr<std::list<LoopConstrain>>
MutliSensorsFusion::getLoopConstraints() {
  std::lock_guard<std::recursive_mutex> lck_g(loop_constrain_mutex_);
  return ptr_l_loop_constrain_;
}

Mat34d MutliSensorsFusion::getLastNodePose() {
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  return ptr_l_node_->back().ptr_keyframe->T_wb_;
}

Mat34d MutliSensorsFusion::getLastOdomPose() {
  std::lock_guard<std::recursive_mutex> lck_g(graph_mutex_);
  // return ptr_l_node_->back().ptr_keyframe->getWheelPose();
  return ptr_l_node_->back().ptr_keyframe->getLaserOdomPose();
}

}  // namespace cvte_lidar_slam
