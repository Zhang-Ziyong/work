#include "common/task.hpp"

namespace slam2d_core {
namespace common {

Task::~Task() {
  // TODO(gaschler): Relax some checks after testing.
  if (state_ != NEW && state_ != COMPLETED) {
    LOG(WARNING) << "Delete Task between dispatch and completion.";
  }
}

Task::State Task::getState() {
  MutexLocker locker(&mutex_);
  return state_;
}

void Task::setWorkItem(const WorkItem &work_item) {
  MutexLocker locker(&mutex_);
  CHECK_EQ(state_, NEW);
  work_item_ = work_item;
}

void Task::addDependency(std::weak_ptr<Task> dependency) {
  std::shared_ptr<Task> shared_dependency;
  {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, NEW);
    if ((shared_dependency = dependency.lock())) {
      ++uncompleted_dependencies_;
    }
  }
  if (shared_dependency) {
    shared_dependency->addDependentTask(this);
  }
}

void Task::setThreadPool(ThreadPool *thread_pool) {
  MutexLocker locker(&mutex_);
  CHECK_EQ(state_, NEW);
  state_ = DISPATCHED;
  thread_pool_to_notify_ = thread_pool;
  if (uncompleted_dependencies_ == 0) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->notifyDependenciesCompleted(this);
  }
}

void Task::addDependentTask(Task *dependent_task) {
  MutexLocker locker(&mutex_);
  if (state_ == COMPLETED) {
    dependent_task->onDependenyCompleted();
    return;
  }
  bool inserted = dependent_tasks_.insert(dependent_task).second;
  CHECK(inserted) << "Given dependency is already a dependency.";
}

void Task::onDependenyCompleted() {
  MutexLocker locker(&mutex_);
  CHECK(state_ == NEW || state_ == DISPATCHED);
  --uncompleted_dependencies_;
  if (uncompleted_dependencies_ == 0 && state_ == DISPATCHED) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->notifyDependenciesCompleted(this);
  }
}

void Task::execute() {
  {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, DEPENDENCIES_COMPLETED);
    state_ = RUNNING;
  }

  // Execute the work item.
  if (work_item_) {
    work_item_();
  }

  MutexLocker locker(&mutex_);
  state_ = COMPLETED;
  for (Task *dependent_task : dependent_tasks_) {
    dependent_task->onDependenyCompleted();
  }
}

}  // namespace common
}  // namespace slam2d_core