#ifndef _THREADED_QUEUE
#define _THREADED_QUEUE



#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>

//
//https://github.com/ToniRV/threadsafe_queue/blob/master/include/ThreadsafeQueue.h
template <typename T>
class ThreadsafeQueue {
public:
  ThreadsafeQueue() {}
  ThreadsafeQueue(const ThreadsafeQueue& other) {
    std::unique_lock<std::mutex> lk(other.mutex_);
    data_queue_ = other.data_queue_;
    lk.unlock();
    // No need to lock here, atomic bool.
    shutdown_ = other.shutdown_;
  }

  // Push an lvalue to the queue.
  // Returns false if the queue has been shutdown.
  bool push(const T& new_value) {
    if (shutdown_) return false; // atomic, no lock needed.
    std::unique_lock<std::mutex> lk(mutex_);
    data_queue_.push(new_value);
    lk.unlock(); // Unlock before notify.
    data_cond_.notify_one();
    return true;
  }

  // Push rvalue to the queue using move semantics.
  // Returns false if the queue has been shutdown.
  // Since there is no type deduction, T&& is an rvalue, not a universal
  // reference.
  bool push(T&& new_value) {
    if (shutdown_) return false; // atomic, no lock needed.
    std::unique_lock<std::mutex> lk(mutex_);
    data_queue_.push(std::move(new_value));
    lk.unlock(); // Unlock before notify.
    data_cond_.notify_one();
    return true;
  }

  // Pop value. Waits for data to be available in the queue.
  // Returns false if the queue has been shutdown.
  bool popBlocking(T& value) {
    std::unique_lock<std::mutex> lk(mutex_);
    // Wait until there is data in the queue or shutdown requested.
    data_cond_.wait(lk, [this]{return !data_queue_.empty() || shutdown_;});
    // Return false in case shutdown is requested.
    if (shutdown_) return false;
    value = data_queue_.front();
    data_queue_.pop();
    return true;
  }

  // Pop value. Waits for data to be available in the queue.
  // If the queue has been shutdown, it returns a null shared_ptr.
  std::shared_ptr<T> popBlocking() {
    std::unique_lock<std::mutex> lk(mutex_);
    data_cond_.wait(lk,[this]{return !data_queue_.empty() || shutdown_;});
    if (shutdown_) return std::shared_ptr<T>();
    // The shared_ptr allocation might throw an exception.
    // Making the queue hold shared_ptr instead, would avoid this issue.
    // See listing 6.3 in [1].
    std::shared_ptr<T> result(std::make_shared<T>(data_queue_.front()));
    data_queue_.pop();
    return result;
  }

  // Pop without blocking, just checks once if the queue is empty.
  // Returns true if the value could be retrieved, false otherwise.
  bool pop(T& value) {
    if (shutdown_) return false;
    std::lock_guard<std::mutex> lk(mutex_);
    if (data_queue_.empty()) return false;
    value = data_queue_.front();
    data_queue_.pop();
    return true;
  }

  // Pop without blocking, just checks once if the queue is empty.
  // Returns a shared_ptr to the value retrieved.
  // If the queue is empty or has been shutdown,
  // it returns a null shared_ptr.
  std::shared_ptr<T> pop() {
    if (shutdown_) return std::shared_ptr<T>();
    std::lock_guard<std::mutex> lk(mutex_);
    if(data_queue_.empty()) return std::shared_ptr<T>();
    // The shared_ptr allocation might throw an exception.
    // Making the queue hold shared_ptr instead, would avoid this issue.
    // See listing 6.3 in [1].
    std::shared_ptr<T> result(std::make_shared<T>(data_queue_.front()));
    data_queue_.pop();
    return result;
  }

  void shutdown() {
    std::unique_lock<std::mutex> mlock(mutex_);
    // Even if the shared variable is atomic, it must be modified under the
    // mutex in order to correctly publish the modification to the waiting
    // threads.
    shutdown_ = true;
    mlock.unlock();
    data_cond_.notify_all();
  }

  void resume() {
    std::unique_lock<std::mutex> mlock(mutex_);
    // Even if the shared variable is atomic, it must be modified under the
    // mutex in order to correctly publish the modification to the waiting
    // threads.
    shutdown_ = false;
    mlock.unlock();
    data_cond_.notify_all();
  }

private:
  // Checks if the queue is empty.
  // Kept private because it might be misused by the user,
  // since the state of the queue might change right after this query.
  bool empty() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return data_queue_.empty();
  }

private:
  mutable std::mutex mutex_; // mutable for empty() and copy-constructor.
  std::queue<T> data_queue_;
  std::condition_variable data_cond_;
  std::atomic_bool shutdown_ = {false}; // flag for signaling queue shutdown.
};

#endif