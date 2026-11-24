// Read-only transport witness. Pixel payloads are never copied or serialized.
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
volatile std::sig_atomic_t stopped=0;
void stop(int){stopped=1;}
int main(int argc,char **argv){
  if(argc!=2)return 2;
  const std::string folder=argv[1];
  std::ofstream output(folder+"/native-images.jsonl"),status(folder+"/native-images-status.json");
  if(!output || !status)return 3;
  std::mutex mutex;std::condition_variable ready;std::deque<std::string> queue;
  bool done=false,failed=false;size_t received=0,written=0,high=0;
  std::thread writer([&]{
    while(true){
      std::string row;
      {std::unique_lock<std::mutex> lock(mutex);ready.wait(lock,[&]{return done || !queue.empty();});
       if(queue.empty() && done)break;
       row=std::move(queue.front());queue.pop_front();}
      output<<row<<'\n';
      {std::lock_guard<std::mutex> lock(mutex);if(!output){failed=true;stopped=1;}else ++written;}
    }
    output.flush();if(!output){std::lock_guard<std::mutex> lock(mutex);failed=true;}
  });
  gz::transport::Node node;
  auto callback=[&](const std::string &kind,const gz::msgs::Image &image){
    const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto &stamp=image.header().stamp();
    std::ostringstream row;
    row<<"{\"kind\":\""<<kind<<"\",\"stamp_ns\":"<<static_cast<int64_t>(stamp.sec())*1000000000LL+stamp.nsec()
       <<",\"received_wall_ns\":"<<wall<<",\"width\":"<<image.width()<<",\"height\":"<<image.height()<<"}";
    std::lock_guard<std::mutex> lock(mutex);++received;
    if(done || queue.size()>=8192){failed=true;stopped=1;return;}
    queue.push_back(row.str());high=std::max(high,queue.size());ready.notify_one();
  };
  std::function<void(const gz::msgs::Image&)> rgb=[&](const gz::msgs::Image &image){callback("rgb",image);};
  std::function<void(const gz::msgs::Image&)> depth=[&](const gz::msgs::Image &image){callback("depth",image);};
  if(!node.Subscribe("/mooncar/rgb",rgb) || !node.Subscribe("/mooncar/depth",depth)){
    std::lock_guard<std::mutex> lock(mutex);failed=true;stopped=1;
  }
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  while(!stopped)std::this_thread::sleep_for(std::chrono::milliseconds(20));
  node.Unsubscribe("/mooncar/rgb");node.Unsubscribe("/mooncar/depth");
  {std::lock_guard<std::mutex> lock(mutex);done=true;ready.notify_all();}
  writer.join();
  const bool drained=!failed && received==written && queue.empty();
  status<<"{\"received\":"<<received<<",\"written\":"<<written<<",\"high_water\":"<<high
        <<",\"drained\":"<<(drained?"true":"false")<<",\"errors\":"<<(failed?"[\"Native observer queue, subscription or write failure\"]":"[]")<<"}\n";
  status.flush();return drained && status?0:1;
}
