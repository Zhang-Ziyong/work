#include "backend/pose_graph.hpp"
#include <iomanip>

namespace slam2d_core {
namespace backend {

PoseGraph::PoseGraph(const PoseGraphOptions &options,
                     std::unique_ptr<OptimizationProblem> optimization_problem,
                     common::ThreadPool *thread_pool)
    : options_(options),
      constraint_builder_(options_.constraint_builder_options, thread_pool),
      thread_pool_(thread_pool),
      optimization_problem_(std::move(optimization_problem)) {}

PoseGraph::~PoseGraph() {
  waitForAllComputations();
  common::MutexLocker locker(&work_queue_mutex_);
  CHECK(work_queue_ == nullptr);
}

// 获取优化器的submap
// 如果还有没有submap 创建一个id为0的subamp 并添加到优化器，然后返回
// 如果插入的submap和存储的submap最后的一个submap相同，更新优化器的submap，返回最后两个submap
// id 如果是个新的submap,返回最后两个submap id
std::vector<int> PoseGraph::initializeGlobalSubmapPoses(
    const common::Time time,
    const std::vector<std::shared_ptr<const frontend::Submap2D>>
        &insertion_submaps) {
  (void) time;
  CHECK(!insertion_submaps.empty());
  const auto &submap_data = optimization_problem_->submap_data();

  // 继续构图
  if (return_mapping_ && (insertion_submaps.size() == 1)) {
    if ((return_mapping_nodeid_ + 1) ==
        data_.trajectory_nodes.rbegin()->first) {
      optimization_problem_->addSubmap(common::project2D(
          return_mapping_pose_ * insertion_submaps[0]->local_pose()));
    }
    return {std::prev(data_.submap_data.end())->first};
  }

  if (insertion_submaps.size() == 1) {
    if (0 == submap_data.size()) {
      optimization_problem_->addSubmap(common::project2D(
          computeLocalToGlobalTransform(data_.global_submap_poses_2d) *
          insertion_submaps[0]->local_pose()));
    }

    CHECK_EQ(1, submap_data.size());
    const int submap_id = 0;
    CHECK(data_.submap_data.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }

  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.end();
  CHECK(submap_data.begin() != end_it);
  const int last_submap_id = std::prev(end_it)->first;

  if (data_.submap_data.at(last_submap_id).submap ==
      insertion_submaps.front()) {
    const auto &first_submap_pose = submap_data.at(last_submap_id).global_pose;

    optimization_problem_->addSubmap(
        first_submap_pose * computeSubmapPose(*insertion_submaps[0]).inverse() *
        computeSubmapPose(*insertion_submaps[1]));

    // LOG(INFO) << "first_submap_pose: " << first_submap_pose.translation().x()
    //           << ", " << first_submap_pose.translation().y();
    return {last_submap_id, last_submap_id + 1};
  }

  CHECK(data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.back());
  const int front_submap_id = last_submap_id - 1;
  CHECK(data_.submap_data.at(front_submap_id).submap ==
        insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

void PoseGraph::addOdometryData(const common::OdometryData &odometry_data) {
  addWorkItem([=]() REQUIRES(mutex_) {
    common::MutexLocker locker(&mutex_);

    if (canAddWorkItemModifying()) {
      optimization_problem_->addOdometryData(odometry_data);
    }
    return common::WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph::addImuData(const common::ImuData &imu_data) {
  addWorkItem([=]() REQUIRES(mutex_) {
    common::MutexLocker locker(&mutex_);

    if (canAddWorkItemModifying()) {
      optimization_problem_->addImuData(imu_data);
    }
    return common::WorkItem::Result::kDoNotRunOptimization;
  });
}

// 生成node节点  并且插入submap
// 定位采样器，构图中好像没用
//
int PoseGraph::appendNode(
    std::shared_ptr<const common::TrajectoryNode::Data> constant_data,
    const std::vector<std::shared_ptr<const frontend::Submap2D>>
        &insertion_submaps,
    const common::Rigid3 &optimized_pose) {
  common::MutexLocker locker(&mutex_);

  if (nullptr == global_localization_sampler_) {
    global_localization_sampler_ = std::make_unique<common::FixedRatioSampler>(
        options_.global_sampling_ratio);
  }

  if (!canAddWorkItemModifying()) {
    LOG(WARNING) << "AddNode was called for finished or deleted trajectory.";
  }

  const int node_id = data_.trajectory_nodes.empty()
                          ? 0
                          : data_.trajectory_nodes.rbegin()->first + 1;

  double move_distance = 0.0;
  if (0 != node_id) {
    if (!return_mapping_) {
      auto local_delta = data_.trajectory_nodes.rbegin()
                             ->second.constant_data->local_pose.inverse() *
                         constant_data->local_pose;
      move_distance =
          data_.trajectory_nodes.rbegin()->second.constant_data->move_distance +
          fabs(local_delta.translation().norm());
    } else {
      auto global_delta =
          data_.trajectory_nodes.rbegin()->second.global_pose.inverse() *
          optimized_pose;
      move_distance =
          data_.trajectory_nodes.rbegin()->second.constant_data->move_distance +
          fabs(global_delta.translation().norm());
    }
  }

  std::shared_ptr<const common::TrajectoryNode::Data> constant =
      std::make_shared<const common::TrajectoryNode::Data>(
          common::TrajectoryNode::Data{
              constant_data->time, constant_data->point_cloud,
              constant_data->local_pose, move_distance});

  data_.trajectory_nodes.emplace(
      node_id, common::TrajectoryNode{constant, optimized_pose});

  ++data_.num_trajectory_nodes;

  if (data_.submap_data.size() == 0 ||
      std::prev(data_.submap_data.end())->second.submap !=
          insertion_submaps.back()) {
    const int submap_id =
        data_.submap_data.empty() ? 0 : data_.submap_data.rbegin()->first + 1;
    data_.submap_data.emplace(submap_id, InternalSubmapData());
    data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    LOG(INFO) << "Inserted submap " << submap_id << ".";
  }
  return node_id;
}

// int->node id.
int PoseGraph::addNode(
    std::shared_ptr<const common::TrajectoryNode::Data> constant_data,
    const std::vector<std::shared_ptr<const frontend::Submap2D>>
        &insertion_submaps) {
  const bool newly_finished_submap =
      insertion_submaps.front()->insertion_finished();
  if (newly_finished_submap) {
    return_mapping_ = false;
  }

  const common::Rigid3 optimized_pose(getLocalToGlobalTransform() *
                                      constant_data->local_pose);
  const int node_id =
      appendNode(constant_data, insertion_submaps, optimized_pose);

  addWorkItem([=]() REQUIRES(mutex_) {
    return computeConstraintsForNode(node_id, insertion_submaps,
                                     newly_finished_submap);
  });
  return node_id;
}

// 计算约束
// 输入最新的node id 以及submap
// 获取两个最新的submap
// 增加优化器的轨迹节点
// 在两个最新的submap增加新的node id
// 保存完成的subamp
// 如果新完成的submap，修改状态为finish
// 计算新的node id 和所有已完成的submap 的约束
// 如果是新完成的submap, 计算所有node id 和新完成的submap 的约束
// 此node约束构建完
// node计数 并进行全局优化(回环)

common::WorkItem::Result PoseGraph::computeConstraintsForNode(
    const int &node_id,
    std::vector<std::shared_ptr<const frontend::Submap2D>> insertion_submaps,
    const bool newly_finished_submap) {
  std::vector<int> submap_ids;
  std::vector<int> finished_submap_ids;
  std::set<int> newly_finished_submap_node_ids;
  {
    common::MutexLocker locker(&mutex_);
    const auto &constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;

    submap_ids =
        initializeGlobalSubmapPoses(constant_data->time, insertion_submaps);

    CHECK_EQ(submap_ids.size(), insertion_submaps.size());

    const int matching_id = submap_ids.front();
    const common::Rigid2 local_pose_2d =
        common::project2D(constant_data->local_pose);

    const common::Rigid2 global_pose_2d =
        optimization_problem_->submap_data().at(matching_id).global_pose *
        computeSubmapPose(*insertion_submaps.front()).inverse() * local_pose_2d;

    optimization_problem_->addTrajectoryNode(
        NodeSpec2D{constant_data->time, local_pose_2d, global_pose_2d});

    for (size_t i = 0; i < insertion_submaps.size(); ++i) {
      const int submap_id = submap_ids[i];
      CHECK(data_.submap_data.at(submap_id).state ==
            SubmapState::kNoConstraintSearch);
      // 在submap 增加新的node id
      data_.submap_data.at(submap_id).node_ids.emplace(node_id);
      const common::Rigid2 constraint_transform =
          computeSubmapPose(*insertion_submaps[i]).inverse() * local_pose_2d;
      data_.constraints.push_back(
          Constraint{submap_id,
                     node_id,
                     {common::embed3D(constraint_transform),
                      options_.matcher_translation_weight,
                      options_.matcher_rotation_weight},
                     Constraint::INTRA_SUBMAP});
    }

    for (const auto &submap_id_data : data_.submap_data) {
      if (submap_id_data.second.state == SubmapState::kFinished) {
        CHECK_EQ(submap_id_data.second.node_ids.count(node_id), 0);
        finished_submap_ids.emplace_back(submap_id_data.first);
      }
    }
    if (newly_finished_submap) {
      const int newly_finished_submap_id = submap_ids.front();
      InternalSubmapData &finished_submap_data =
          data_.submap_data.at(newly_finished_submap_id);
      CHECK(finished_submap_data.state == SubmapState::kNoConstraintSearch);
      finished_submap_data.state = SubmapState::kFinished;
      newly_finished_submap_node_ids = finished_submap_data.node_ids;
    }
  }

  for (const auto &submap_id : finished_submap_ids) {
    computeConstraint(node_id, submap_id);
  }

  if (newly_finished_submap) {
    // LOG(INFO) << "newly_finished_submap";
    const int newly_finished_submap_id = submap_ids.front();
    for (const auto &node_id_data : optimization_problem_->node_data()) {
      const int &node_id = node_id_data.first;
      if (newly_finished_submap_node_ids.count(node_id) == 0) {
        computeConstraint(node_id, newly_finished_submap_id);
      }
    }
  }

  constraint_builder_.notifyEndOfNode();
  common::MutexLocker locker(&mutex_);
  ++num_nodes_since_last_loop_closure_;
  if (options_.optimize_every_n_nodes > 0 &&
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes) {
    return common::WorkItem::Result::kRunOptimization;
  }
  return common::WorkItem::Result::kDoNotRunOptimization;
}

void PoseGraph::computeConstraint(const int &node_id, const int &submap_id) {
  const common::TrajectoryNode::Data *constant_data;
  const frontend::Submap2D *submap;
  {
    common::MutexLocker locker(&mutex_);
    CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
    if (!data_.submap_data.at(submap_id).submap->insertion_finished()) {
      LOG(INFO) << "submap is not insertion finished.";
      return;
    }

    constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    submap = static_cast<const frontend::Submap2D *>(
        data_.submap_data.at(submap_id).submap.get());
  }

  double distance;
  int submap_start_node_id = *data_.submap_data.at(submap_id).node_ids.begin();

  if (0 != data_.trajectory_nodes.count(submap_start_node_id)) {
    distance =
        fabs(data_.trajectory_nodes.at(node_id).constant_data->move_distance -
             data_.trajectory_nodes.at(submap_start_node_id)
                 .constant_data->move_distance);
  } else {
    distance = 100.0;
  }

  const common::Rigid2 initial_relative_pose =
      optimization_problem_->submap_data().at(submap_id).global_pose.inverse() *
      optimization_problem_->node_data().at(node_id).global_pose_2d;
  constraint_builder_.maybeAddConstraint(submap_id, submap, node_id,
                                         constant_data, initial_relative_pose,
                                         distance);
}

common::Time PoseGraph::getLatestNodeTime(const int &node_id,
                                          const int &submap_id) const {
  common::Time time = data_.trajectory_nodes.at(node_id).constant_data->time;
  const InternalSubmapData &submap_data = data_.submap_data.at(submap_id);
  if (!submap_data.node_ids.empty()) {
    const int last_submap_node_id =
        *data_.submap_data.at(submap_id).node_ids.rbegin();
    time = std::max(
        time,
        data_.trajectory_nodes.at(last_submap_node_id).constant_data->time);
  }
  return time;
}

void PoseGraph::addWorkItem(
    const std::function<common::WorkItem::Result()> &work_item) {
  common::MutexLocker locker(&work_queue_mutex_);

  if (work_queue_ == nullptr) {
    work_queue_ = std::make_unique<common::WorkQueue>();
    auto task = std::make_unique<common::Task>();
    task->setWorkItem([this]() { drainWorkQueue(); });
    thread_pool_->schedule(std::move(task));
  }
  const auto now = std::chrono::steady_clock::now();
  work_queue_->push_back({now, work_item});
}

void PoseGraph::drainWorkQueue() {
  bool process_work_queue = true;
  size_t work_queue_size;
  while (process_work_queue) {
    std::function<common::WorkItem::Result()> work_item;
    {
      common::MutexLocker locker(&work_queue_mutex_);
      if (work_queue_->empty()) {
        work_queue_.reset();
        return;
      }
      work_item = work_queue_->front().task;
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
    }
    process_work_queue =
        work_item() == common::WorkItem::Result::kDoNotRunOptimization;
  }

  LOG(INFO) << "Remaining work items in queue: " << work_queue_size;
  // We have to optimize again.
  constraint_builder_.whenDone([this](const ConstraintBuilder::Result &result) {
    handleWorkQueue(result);
  });
}

void PoseGraph::handleWorkQueue(const ConstraintBuilder::Result &result) {
  {
    common::MutexLocker locker(&mutex_);
    data_.constraints.insert(data_.constraints.end(), result.begin(),
                             result.end());
  }
  runOptimization();
  if (global_slam_optimization_callback_) {
    int last_optimized_node_id;
    int last_optimized_submap_id;
    {
      common::MutexLocker locker(&mutex_);
      const auto &submap_data = optimization_problem_->submap_data();
      const auto &node_data = optimization_problem_->node_data();

      last_optimized_node_id = std::prev(node_data.end())->first;
      last_optimized_submap_id = std::prev(submap_data.end())->first;
    }
    global_slam_optimization_callback_(last_optimized_submap_id,
                                       last_optimized_node_id);
  }
  {
    common::MutexLocker locker(&mutex_);
    num_nodes_since_last_loop_closure_ = 0;
  }

  if (is_trim_data_) {
    trimData();
  }

  drainWorkQueue();
}

void PoseGraph::runOptimization() {
  if (optimization_problem_->submap_data().empty()) {
    return;
  }
  //   LOG(INFO) << "data_.constraints size " << data_.constraints.size();
  optimization_problem_->solve(data_.constraints);
  common::MutexLocker locker(&mutex_);

  const auto &submap_data = optimization_problem_->submap_data();
  const auto &node_data = optimization_problem_->node_data();

  // 更新node global pose
  for (const auto &node : node_data) {
    auto &mutable_trajectory_node = data_.trajectory_nodes.at(node.first);
    mutable_trajectory_node.global_pose =
        common::embed3D(node.second.global_pose_2d);
  }

  const auto local_to_new_global = computeLocalToGlobalTransform(submap_data);
  const auto local_to_old_global =
      computeLocalToGlobalTransform(data_.global_submap_poses_2d);

  const common::Rigid3 old_global_to_new_global =
      local_to_new_global * local_to_old_global.inverse();

  const int last_optimized_node_id = std::prev(node_data.end())->first;

  auto node_it = std::next(data_.trajectory_nodes.find(last_optimized_node_id));

  for (; node_it != data_.trajectory_nodes.end(); ++node_it) {
    auto &mutable_trajectory_node = data_.trajectory_nodes.at(node_it->first);
    mutable_trajectory_node.global_pose =
        old_global_to_new_global * mutable_trajectory_node.global_pose;
  }

  data_.global_submap_poses_2d = submap_data;
}

InternalTrajectoryState PoseGraph::getTrajectoryStates() const {
  InternalTrajectoryState trajectory_state;
  common::MutexLocker locker(&mutex_);
  trajectory_state = data_.trajectories_state;
  return trajectory_state;
}

bool PoseGraph::canAddWorkItemModifying() {
  auto it = data_.trajectories_state;
  if (it.state == TrajectoryState::FINISHED) {
    LOG(FATAL) << "trajectory_ has finished but modification is requested, "
                  "skipping.";
    return false;
  }

  if (it.deletion_state != InternalTrajectoryState::DeletionState::NORMAL) {
    LOG(FATAL) << "trajectory  has been scheduled for deletion but "
                  "modification is requested, skipping.";
    return false;
  }

  if (it.state == TrajectoryState::DELETED) {
    LOG(FATAL) << "trajectory_id  has been deleted "
                  "but modification is requested, skipping.";
    return false;
  }
  return true;
}

common::Rigid3 PoseGraph::getLocalToGlobalTransform() const {
  common::MutexLocker locker(&mutex_);
  if (return_mapping_) {
    return return_mapping_pose_;
  } else {
    return computeLocalToGlobalTransform(data_.global_submap_poses_2d);
  }
}

common::Rigid3 PoseGraph::computeLocalToGlobalTransform(
    const std::map<int, SubmapSpec2D> &global_submap_poses) const {
  auto begin_it = global_submap_poses.begin();
  auto end_it = global_submap_poses.end();

  if (begin_it == end_it) {
    // const auto it = data_.initial_trajectory_pose;
    // return getInterpolatedGlobalTrajectoryPose(it.time) * it.relative_pose;

    return common::Rigid3::Identity();
  }

  const int last_optimized_submap_id = std::prev(end_it)->first;

  return common::embed3D(
             global_submap_poses.at(last_optimized_submap_id).global_pose) *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

common::Rigid3 PoseGraph::getInterpolatedGlobalTrajectoryPose(
    const common::Time time) const {
  CHECK_GT(data_.trajectory_nodes.size(), 0);

  auto it = data_.trajectory_nodes.begin();
  if (data_.trajectory_nodes.end()->second.time() < time) {
    it = data_.trajectory_nodes.end();
  } else {
    auto left = data_.trajectory_nodes.begin();
    auto right = std::prev(data_.trajectory_nodes.end());
    while (left != right) {
      const int middle = left->first + (right->first - left->first) / 2;
      auto lower_bound_middle = data_.trajectory_nodes.lower_bound(middle);
      if (lower_bound_middle->first > middle) {
        CHECK(lower_bound_middle != left);
        lower_bound_middle = std::prev(lower_bound_middle);
      }
      if (lower_bound_middle->second.time() < time) {
        left = std::next(lower_bound_middle);
      } else {
        right = lower_bound_middle;
      }
    }
    it = left;
  }

  if (it == data_.trajectory_nodes.begin()) {
    return data_.trajectory_nodes.begin()->second.global_pose;
  }

  if (it == data_.trajectory_nodes.end()) {
    return std::prev(data_.trajectory_nodes.end())->second.global_pose;
  }

  return common::interpolate(
             common::TimestampedTransform{std::prev(it)->second.time(),
                                          std::prev(it)->second.global_pose},
             common::TimestampedTransform{it->second.time(),
                                          it->second.global_pose},
             time)
      .transform;
}

void PoseGraph::addNodeToSubmap(const int &node_id, const int &submap_id) {
  addWorkItem([this, node_id, submap_id]() REQUIRES(mutex_) {
    common::MutexLocker locker(&mutex_);
    if (canAddWorkItemModifying()) {
      data_.submap_data.at(submap_id).node_ids.insert(node_id);
    }
    return common::WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph::finishTrajectory() {
  addWorkItem([this]() REQUIRES(mutex_) {
    common::MutexLocker locker(&mutex_);
    CHECK(!isTrajectoryFinished());
    data_.trajectories_state.state = TrajectoryState::FINISHED;

    for (const auto &submap : data_.submap_data) {
      data_.submap_data.at(submap.first).state = SubmapState::kFinished;
    }
    return common::WorkItem::Result::kRunOptimization;
  });
}

bool PoseGraph::isTrajectoryFinished() const {
  return data_.trajectories_state.state == TrajectoryState::FINISHED;
}

void PoseGraph::runFinalOptimization() {
  {
    addWorkItem([this]() REQUIRES(mutex_) {
      common::MutexLocker locker(&mutex_);

      optimization_problem_->setMaxNumIterations(
          options_.max_num_final_iterations);
      return common::WorkItem::Result::kRunOptimization;
    });
    addWorkItem([this]() REQUIRES(mutex_) {
      common::MutexLocker locker(&mutex_);
      optimization_problem_->setMaxNumIterations(
          options_.optimization_problem_options.ceres_solver_options
              .max_num_iterations);
      return common::WorkItem::Result::kDoNotRunOptimization;
    });
  }
  waitForAllComputations();
}

void PoseGraph::waitForAllComputations() {
  int num_trajectory_nodes;
  {
    common::MutexLocker locker(&mutex_);
    num_trajectory_nodes = data_.num_trajectory_nodes;
  }

  const int num_finished_nodes_at_start =
      constraint_builder_.getNumFinishedNodes();

  auto report_progress = [this, num_trajectory_nodes,
                          num_finished_nodes_at_start]() {
    if (num_trajectory_nodes != num_finished_nodes_at_start) {
      std::ostringstream progress_info;
      progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                    << 100. *
                           (constraint_builder_.getNumFinishedNodes() -
                            num_finished_nodes_at_start) /
                           (num_trajectory_nodes - num_finished_nodes_at_start)
                    << "%...";
      std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
    }
  };

  {
    const auto predicate = [this]() REQUIRES(work_queue_mutex_) {
      return work_queue_ == nullptr;
    };
    common::MutexLocker locker(&work_queue_mutex_);
    while (!locker.AwaitWithTimeout(predicate, common::fromSeconds(1.))) {
      report_progress();
    }
  }

  // Now wait for any pending constraint computations to finish.
  common::MutexLocker locker(&mutex_);

  bool notification = false;
  constraint_builder_.whenDone(
      [this, &notification](const ConstraintBuilder::Result &result)
          EXCLUDES(mutex_) {
            common::MutexLocker locker(&mutex_);
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());
            notification = true;
          });
  const auto predicate = [&notification]()
                             REQUIRES(mutex_) { return notification; };

  while (!locker.AwaitWithTimeout(predicate, common::fromSeconds(1.))) {
    report_progress();
  }
  CHECK_EQ(constraint_builder_.getNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
}

SubmapData PoseGraph::getSubmapData(const int &submap_id) const {
  common::MutexLocker locker(&mutex_);
  return getSubmapDataUnderLock(submap_id);
}

SubmapData PoseGraph::getSubmapDataUnderLock(const int &submap_id) const {
  const auto it = data_.submap_data.find(submap_id);
  if (it == data_.submap_data.end()) {
    return {};
  }
  auto submap = it->second.submap;
  if (data_.global_submap_poses_2d.count(submap_id)) {
    return {submap,
            common::embed3D(
                data_.global_submap_poses_2d.at(submap_id).global_pose)};
  }

  if (return_mapping_) {
    return {submap, return_mapping_pose_ * submap->local_pose()};
  }

  return {submap, computeLocalToGlobalTransform(data_.global_submap_poses_2d) *
                      submap->local_pose()};
}

// int -> node id
std::map<int, common::TrajectoryNode> PoseGraph::getTrajectoryNodes() const {
  common::MutexLocker locker(&mutex_);
  return data_.trajectory_nodes;
}

// int -> node id
std::map<int, common::TrajectoryNodePose> PoseGraph::getTrajectoryNodePoses()
    const {
  std::map<int, common::TrajectoryNodePose> node_poses;
  common::MutexLocker locker(&mutex_);
  for (const auto &node_id_data : data_.trajectory_nodes) {
    std::optional<common::TrajectoryNodePose::ConstantPoseData>
        constant_pose_data;

    if (node_id_data.second.constant_data != nullptr) {
      constant_pose_data = common::TrajectoryNodePose::ConstantPoseData{
          node_id_data.second.constant_data->time,
          node_id_data.second.constant_data->local_pose};
    }
    node_poses.emplace(node_id_data.first, common::TrajectoryNodePose{
                                               node_id_data.second.global_pose,
                                               constant_pose_data});
  }
  return node_poses;
}

std::vector<Constraint> PoseGraph::constraints() const {
  std::vector<Constraint> result;
  common::MutexLocker locker(&mutex_);
  for (const Constraint &constraint : data_.constraints) {
    result.push_back(
        Constraint{constraint.submap_id, constraint.node_id,
                   Constraint::Pose{constraint.pose.zbar_ij,
                                    constraint.pose.translation_weight,
                                    constraint.pose.rotation_weight},
                   constraint.tag});
  }
  return result;
}

std::map<int, SubmapData> PoseGraph::getAllSubmapData() const {
  common::MutexLocker locker(&mutex_);
  return getSubmapDataUnderLock();
}

std::map<int, SubmapData> PoseGraph::getSubmapDataUnderLock() const {
  std::map<int, SubmapData> submaps;
  for (const auto &submap_id_data : data_.submap_data) {
    submaps.emplace(submap_id_data.first,
                    getSubmapDataUnderLock(submap_id_data.first));
  }
  return submaps;
}

std::map<int, SubmapPose> PoseGraph::getAllSubmapPoses() const {
  common::MutexLocker locker(&mutex_);
  std::map<int, SubmapPose> submap_poses;
  for (const auto &submap_id_data : data_.submap_data) {
    auto submap_data = getSubmapDataUnderLock(submap_id_data.first);
    submap_poses.emplace(
        submap_id_data.first,
        SubmapPose{submap_data.submap->num_range_data(), submap_data.pose});
  }
  return submap_poses;
}

std::map<common::Time, common::OdometryData> PoseGraph::getOdometryData()
    const {
  common::MutexLocker locker(&mutex_);
  return optimization_problem_->odometry_data();
}

void PoseGraph::setReturnMappingPose(const common::Rigid3 &global_pose) {
  return_mapping_ = true;
  return_mapping_pose_ = global_pose;
  auto last_node_id = data_.trajectory_nodes.rbegin()->first;
  auto last_node_pose = data_.trajectory_nodes.rbegin()->second.global_pose;
  auto transform = last_node_pose.inverse() * return_mapping_pose_;
  optimization_problem_->setReturnMappingData(last_node_id, transform);
  return_mapping_nodeid_ = last_node_id;
  return_mapping_submapid_ = data_.submap_data.rbegin()->first;
  LOG(INFO) << "last node id: " << last_node_id
            << "last submap id: " << return_mapping_submapid_;
}

void PoseGraph::addSerializedConstraints(
    const std::vector<Constraint> &constraints) {
  addWorkItem([this, constraints]() REQUIRES(mutex_) {
    common::MutexLocker locker(&mutex_);
    for (const auto &constraint : constraints) {
      CHECK((data_.trajectory_nodes.count(constraint.node_id) != 0));
      CHECK((data_.submap_data.count(constraint.submap_id) != 0));
      CHECK(data_.trajectory_nodes.at(constraint.node_id).constant_data !=
            nullptr);

      CHECK(data_.submap_data.at(constraint.submap_id).submap != nullptr);

      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
          CHECK(data_.submap_data.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          // updateTrajectoryConnectivity(constraint);
          break;
      }

      const Constraint::Pose pose = {constraint.pose.zbar_ij,
                                     constraint.pose.translation_weight,
                                     constraint.pose.rotation_weight};

      data_.constraints.push_back(Constraint{
          constraint.submap_id, constraint.node_id, pose, constraint.tag});
    }
    LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
    return common::WorkItem::Result::kDoNotRunOptimization;
  });
}

Constraint::Tag fromProto(
    const slam2d::mapping::proto::PoseGraph::Constraint::Tag &proto) {
  switch (proto) {
    case slam2d::mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP:
      return Constraint::Tag::INTRA_SUBMAP;
    case slam2d::mapping::proto::PoseGraph::Constraint::INTER_SUBMAP:
      return Constraint::Tag::INTER_SUBMAP;
    case ::google::protobuf::kint32max:
    case ::google::protobuf::kint32min: {
    }
  }

  LOG(FATAL) << "Unsupported tag.";
}

slam2d::mapping::proto::PoseGraph::Constraint::Tag toProto(
    const Constraint::Tag &tag) {
  switch (tag) {
    case Constraint::Tag::INTRA_SUBMAP:
      return slam2d::mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP;
    case Constraint::Tag::INTER_SUBMAP:
      return slam2d::mapping::proto::PoseGraph::Constraint::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag.";
}

slam2d::mapping::proto::PoseGraph::Constraint toProto(
    const Constraint &constraint) {
  slam2d::mapping::proto::PoseGraph::Constraint constraint_proto;
  *constraint_proto.mutable_relative_pose() =
      common::toProto(constraint.pose.zbar_ij);

  constraint_proto.set_translation_weight(constraint.pose.translation_weight);
  constraint_proto.set_rotation_weight(constraint.pose.rotation_weight);
  constraint_proto.set_submap_id(constraint.submap_id);
  constraint_proto.set_node_id(constraint.node_id);

  constraint_proto.set_tag(toProto(constraint.tag));

  return constraint_proto;
}

slam2d::mapping::proto::PoseGraph PoseGraph::toProto(
    bool include_unfinished_submaps) const {
  slam2d::mapping::proto::PoseGraph proto;

  slam2d::mapping::proto::Trajectory trajectory_proto;

  std::set<int> unfinished_submaps;
  for (const auto &submap_id_data : getAllSubmapData()) {
    if (!include_unfinished_submaps &&
        !submap_id_data.second.submap->insertion_finished()) {
      unfinished_submaps.insert(submap_id_data.first);
      continue;
    }
    CHECK(submap_id_data.second.submap != nullptr);

    auto *const submap_proto = trajectory_proto.add_submap();

    submap_proto->set_submap_index(submap_id_data.first);
    *submap_proto->mutable_pose() = common::toProto(submap_id_data.second.pose);
  }

  auto constraints_copy = constraints();

  std::set<int> orphaned_nodes;
  proto.mutable_constraint()->Reserve(constraints_copy.size());
  for (auto it = constraints_copy.begin(); it != constraints_copy.end();) {
    if (!include_unfinished_submaps &&
        unfinished_submaps.count(it->submap_id) > 0) {
      orphaned_nodes.insert(it->node_id);
      it = constraints_copy.erase(it);
      continue;
    }
    *proto.add_constraint() = backend::toProto(*it);
    ++it;
  }

  if (!include_unfinished_submaps) {
    for (const auto &constraint : constraints_copy) {
      orphaned_nodes.erase(constraint.node_id);
    }
  }

  for (const auto &node_id_data : getTrajectoryNodes()) {
    CHECK(node_id_data.second.constant_data != nullptr);
    auto *const node_proto = trajectory_proto.add_node();
    node_proto->set_node_index(node_id_data.first);
    node_proto->set_timestamp(
        common::toUniversal(node_id_data.second.constant_data->time));
    *node_proto->mutable_pose() =
        common::toProto(node_id_data.second.global_pose);
  }

  *proto.mutable_trajectory() = trajectory_proto;

  return proto;
}

void PoseGraph::addSubmapFromProto(
    const common::Rigid3 &global_submap_pose,
    const slam2d::mapping::proto::SubmapData &submap) {
  if (!submap.has_submap_data()) {
    return;
  }

  int submap_id = submap.submap_id();

  const common::Rigid2 global_submap_pose_2d =
      common::project2D(global_submap_pose);
  {
    common::MutexLocker locker(&mutex_);

    const std::shared_ptr<const frontend::Submap2D> submap_ptr =
        std::make_shared<const frontend::Submap2D>(submap.submap_data(),
                                                   &conversion_tables_);
    if (!canAddWorkItemModifying()) {
      return;
    }
    data_.submap_data.emplace(submap_id, InternalSubmapData());
    data_.submap_data.at(submap_id).submap = submap_ptr;
    data_.global_submap_poses_2d.emplace(submap_id,
                                         SubmapSpec2D{global_submap_pose_2d});
  }

  addWorkItem([this, submap_id, global_submap_pose_2d]() REQUIRES(mutex_) {
    common::MutexLocker locker(&mutex_);
    data_.submap_data.at(submap_id).state = SubmapState::kFinished;
    optimization_problem_->insertSubmap(submap_id, global_submap_pose_2d);
    return common::WorkItem::Result::kDoNotRunOptimization;
  });
}

void PoseGraph::addNodeFromProto(const common::Rigid3 &global_pose,
                                 const slam2d::mapping::proto::Node &node) {
  std::shared_ptr<const common::TrajectoryNode::Data> constant_data =
      std::make_shared<const common::TrajectoryNode::Data>(
          common::fromProto(node.node_data()));

  int node_id = node.node_id();

  {
    common::MutexLocker locker(&mutex_);

    if (!canAddWorkItemModifying()) {
      return;
    }
    data_.trajectory_nodes.emplace(
        node_id, common::TrajectoryNode{constant_data, global_pose});
  }

  addWorkItem([this, node_id, global_pose]() REQUIRES(mutex_) {
    common::MutexLocker locker(&mutex_);
    const auto &constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;

    optimization_problem_->insertTrajectoryNode(
        node_id, NodeSpec2D{constant_data->time,
                            common::project2D(constant_data->local_pose),
                            common::project2D(global_pose)});

    return common::WorkItem::Result::kDoNotRunOptimization;
  });
}

std::vector<Constraint> fromProto(
    const ::google::protobuf::RepeatedPtrField<
        ::slam2d::mapping::proto::PoseGraph::Constraint> &constraint_protos) {
  std::vector<Constraint> constraints;
  for (const auto &constraint_proto : constraint_protos) {
    const int submap_id = constraint_proto.submap_id();
    const int node_id = constraint_proto.node_id();

    const Constraint::Pose pose{
        common::toRigid3(constraint_proto.relative_pose()),
        constraint_proto.translation_weight(),
        constraint_proto.rotation_weight()};
    const Constraint::Tag tag = fromProto(constraint_proto.tag());
    constraints.push_back(Constraint{submap_id, node_id, pose, tag});
  }
  return constraints;
}

void PoseGraph::trimData() {
  std::vector<int> submap_ids;
  const auto &submap_data = optimization_problem_->submap_data();
  for (const auto &it : submap_data) { submap_ids.push_back(it.first); }

  for (size_t i = 0;
       i + options_.pure_localization_submaps_keep < submap_ids.size(); ++i) {
    trimSubmap(submap_ids.at(i));
  }
}

void PoseGraph::trimSubmap(const int &submap_id) {
  CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);

  // 构图时候的submap数据这些不能清除
  if (submap_id <= return_mapping_submapid_) {
    return;
  }

  std::set<int> nodes_to_retain;
  for (const auto &submap_data : data_.submap_data) {
    if (submap_data.first != submap_id) {
      nodes_to_retain.insert(submap_data.second.node_ids.begin(),
                             submap_data.second.node_ids.end());
    }
  }

  std::set<int> nodes_to_remove;
  std::set_difference(data_.submap_data.at(submap_id).node_ids.begin(),
                      data_.submap_data.at(submap_id).node_ids.end(),
                      nodes_to_retain.begin(), nodes_to_retain.end(),
                      std::inserter(nodes_to_remove, nodes_to_remove.begin()));

  {
    std::vector<Constraint> constraints;
    for (const Constraint &constraint : data_.constraints) {
      if (constraint.submap_id != submap_id) {
        constraints.push_back(constraint);
      }
    }
    data_.constraints = std::move(constraints);
  }

  {
    std::vector<Constraint> constraints;
    std::set<int> other_submap_ids_losing_constraints;
    for (const Constraint &constraint : data_.constraints) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      } else {
        other_submap_ids_losing_constraints.insert(constraint.submap_id);
      }
    }
    data_.constraints = std::move(constraints);

    for (const Constraint &constraint : data_.constraints) {
      if (constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
        continue;
      } else if (other_submap_ids_losing_constraints.count(
                     constraint.submap_id)) {
        other_submap_ids_losing_constraints.erase(constraint.submap_id);
      }
    }

    for (const int &submap_id : other_submap_ids_losing_constraints) {
      constraint_builder_.deleteScanMatcher(submap_id);
    }
  }

  CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
  data_.submap_data.erase(submap_id);
  constraint_builder_.deleteScanMatcher(submap_id);
  optimization_problem_->trimSubmap(submap_id);

  for (const int &node_id : nodes_to_remove) {
    data_.trajectory_nodes.erase(node_id);
    optimization_problem_->trimTrajectoryNode(node_id);
  }
}

}  // namespace backend
}  // namespace slam2d_core
