#ifndef _THREAD_POOL_H_
#define _THREAD_POOL_H_

#include "common/task.hpp"
#include "mutex.hpp"
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace slam2d_core {
namespace common {

class Task;

class ThreadPool {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;

  std::weak_ptr<Task> schedule(std::unique_ptr<Task> task) EXCLUDES(mutex_);

 protected:
  void execute(Task *task);
  void setThreadPool(Task *task);

 private:
  friend class Task;
  void doWork();

  void notifyDependenciesCompleted(Task *task) EXCLUDES(mutex_);

  Mutex mutex_;
  bool running_ GUARDED_BY(mutex_) = true;
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);
  std::deque<std::shared_ptr<Task>> task_queue_ GUARDED_BY(mutex_);
  std::unordered_map<Task *, std::shared_ptr<Task>> tasks_not_ready_
      GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace slam2d_core

#endif
