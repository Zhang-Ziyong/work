#ifndef _COMMON_TASK_H_
#define _COMMON_TASK_H_

#include <set>

#include "common/mutex.hpp"
#include "common/thread_pool.hpp"
#include "glog/logging.h"
#include <chrono>
#include <deque>
#include <functional>

namespace slam2d_core {
namespace common {

class ThreadPool;

class Task {
 public:
  friend class ThreadPool;

  using WorkItem = std::function<void()>;
  enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

  Task() = default;
  ~Task();

  State getState() EXCLUDES(mutex_);

  void setWorkItem(const WorkItem &work_item) EXCLUDES(mutex_);

  void addDependency(std::weak_ptr<Task> dependency) EXCLUDES(mutex_);

 private:
  void addDependentTask(Task *dependent_task);

  void execute() EXCLUDES(mutex_);

  void setThreadPool(ThreadPool *thread_pool) EXCLUDES(mutex_);

  void onDependenyCompleted();

  WorkItem work_item_ GUARDED_BY(mutex_);
  ThreadPool *thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;
  State state_ GUARDED_BY(mutex_) = NEW;
  unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;
  std::set<Task *> dependent_tasks_ GUARDED_BY(mutex_);

  Mutex mutex_;
};

struct WorkItem {
  enum class Result {
    kDoNotRunOptimization,
    kRunOptimization,
  };

  std::chrono::steady_clock::time_point time;
  std::function<Result()> task;
};

using WorkQueue = std::deque<WorkItem>;

}  // namespace common
}  // namespace slam2d_core

#endif  // CARTOGRAPHER_COMMON_TASK_H_
