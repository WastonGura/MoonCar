// Test-only header logger. No pixel serialization or control writes.
#pragma once
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
inline int64_t deliveryWall() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
class BoundedLog {
  std::ofstream output;
  std::string path;
  std::mutex mutex;
  std::condition_variable ready;
  std::deque<std::string> queue;
  bool done=false,failed=false;
  size_t received=0,written=0,high=0;
  std::thread worker;
public:
  explicit BoundedLog(const std::string &p):output(p),path(p) {
    if(!output)throw std::runtime_error("Cannot open delivery log");
    worker=std::thread([this]{
      for(;;){
        std::string row;
        {std::unique_lock<std::mutex> lock(mutex);
         ready.wait(lock,[this]{return done || !queue.empty();});
         if(done && queue.empty())break;
         row=std::move(queue.front());queue.pop_front();}
        output<<row<<'\n';
        {std::lock_guard<std::mutex> lock(mutex);
         if(!output)failed=true;else ++written;}
      }
      output.flush();
      if(!output){std::lock_guard<std::mutex> lock(mutex);failed=true;}
    });
  }
  void put(const std::string &row){
    std::lock_guard<std::mutex> lock(mutex);
    if(failed || done || queue.size()>=8192){failed=true;throw std::runtime_error("Delivery writer overflow, exit or write failure");}
    ++received;queue.push_back(row);high=std::max(high,queue.size());ready.notify_one();
  }
  bool finish(){
    {std::lock_guard<std::mutex> lock(mutex);done=true;ready.notify_all();}
    if(worker.joinable())worker.join();
    const bool ok=!failed && queue.empty() && received==written;
    std::ofstream status(path+".status.json");
    status<<"{\"received\":"<<received<<",\"written\":"<<written<<",\"high_water\":"<<high
      <<",\"drained\":"<<(ok?"true":"false")<<",\"failed\":"<<(failed?"true":"false")<<"}\n";
    status.flush();return ok && bool(status);
  }
  ~BoundedLog(){if(worker.joinable())finish();}
};
