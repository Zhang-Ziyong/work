#include "common/thread_pool.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>

#include "glog/logging.h"

namespace slam2d_core {
namespace common {

void ThreadPool::execute(Task *task) {
  task->execute();
}

void ThreadPool::setThreadPool(Task *task) {
  task->setThreadPool(this);
}

ThreadPool::ThreadPool(int num_threads) {
  MutexLocker locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::doWork(); });
  }
}

ThreadPool::~ThreadPool() {
  {
    MutexLocker locker(&mutex_);
    CHECK(running_);
    running_ = false;
  }
  for (std::thread &thread : pool_) { thread.join(); }
}

void ThreadPool::notifyDependenciesCompleted(Task *task) {
  MutexLocker locker(&mutex_);
  auto it = tasks_not_ready_.find(task);
  CHECK(it != tasks_not_ready_.end());
  task_queue_.push_back(it->second);
  tasks_not_ready_.erase(it);
}

std::weak_ptr<Task> ThreadPool::schedule(std::unique_ptr<Task> task) {
  std::shared_ptr<Task> shared_task;
  {
    MutexLocker locker(&mutex_);
    auto insert_result =
        tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
    CHECK(insert_result.second) << "Schedule called twice";
    shared_task = insert_result.first->second;
  }
  setThreadPool(shared_task.get());
  return shared_task;
}

void ThreadPool::doWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  for (;;) {
    std::shared_ptr<Task> task;
    {
      MutexLocker locker(&mutex_);
      locker.Await([this]() REQUIRES(mutex_) {
        return !task_queue_.empty() || !running_;
      });

      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      } else if (!running_) {
        return;
      }
    }
    CHECK(task);
    CHECK_EQ(task->getState(), common::Task::DEPENDENCIES_COMPLETED);
    execute(task.get());
  }
}

}  // namespace common
}  // namespace slam2d_core
