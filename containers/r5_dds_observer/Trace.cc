#include "Trace.hh"
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
namespace {
struct Row {MooncarDdsEvent event;uint64_t wall_ns;};
struct Log {
    std::array<Row,8192> queue{};std::mutex mutex;std::condition_variable wake;
    size_t head=0,tail=0,count=0,peak=0;uint64_t submitted=0,written=0,overflow=0,rejected=0;
    bool stopping=false;std::atomic<bool> failed{false};std::string path;std::thread thread;
    Log(){auto p=std::getenv("MOONCAR_DDS_TRACE");if(p){path=std::string(p)+"."+std::to_string(getpid())+".jsonl";thread=std::thread([this]{write();});}}
    ~Log(){finish();}
    static std::string hex(const uint8_t* p,size_t n){const char* h="0123456789abcdef";std::string s;for(size_t i=0;i<n;++i){s+=h[p[i]>>4];s+=h[p[i]&15];}return s;}
    static std::string identity(const uint8_t* p,size_t n){for(size_t i=0;i<n;++i)if(p[i])return "\""+hex(p,n)+"\"";return "null";}
    void write(){
        // A ROS message binding may load this DSO before the test's output
        // directory is created. Waiting/opening remains outside DDS threads.
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
        const auto parent=std::filesystem::path(path).parent_path();
        while(!std::filesystem::is_directory(parent)&&std::chrono::steady_clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::ofstream file(path);if(!file){std::lock_guard<std::mutex> l(mutex);failed=true;return;}
        for(;;){Row r;{std::unique_lock<std::mutex> l(mutex);wake.wait(l,[this]{return count||stopping;});if(!count&&stopping)break;
            r=queue[tail];tail=(tail+1)%queue.size();--count;}
            auto e=r.event;
            file<<"{\"stage\":\""<<e.stage<<"\",\"pid\":"<<getpid()<<",\"wall_ns\":"<<r.wall_ns
              <<",\"segment\":"<<identity(e.segment,8)<<",\"offset\":"<<e.offset<<",\"validity\":"<<e.validity
              <<",\"writer\":"<<identity(e.writer,16)<<",\"reader\":"<<identity(e.reader,16)<<",\"sequence\":";
            if(e.sequence<0)file<<"null";else file<<e.sequence;
            file<<",\"stamp_ns\":";if(e.stamp_ns<0)file<<"null";else file<<e.stamp_ns;
            file<<",\"image_kind\":"<<((std::strcmp(e.stage,"rmw_publish")==0||std::strcmp(e.stage,"rmw_take")==0||std::strcmp(e.stage,"reader_qos")==0)?(e.fragments==1?"\"rgb\"":e.fragments==2?"\"depth\"":"null"):"null")
                <<",\"fragment\":"<<e.fragment<<",\"fragments\":"<<e.fragments
                <<",\"a\":"<<e.a<<",\"b\":"<<e.b<<",\"c\":"<<e.c<<",\"d\":"<<e.d<<"}\n";
            if(!file){std::lock_guard<std::mutex> l(mutex);failed=true;break;}++written;
        }file.flush();if(!file)failed=true;
    }
    void finish(){if(path.empty())return;if(thread.joinable()){{std::lock_guard<std::mutex> l(mutex);stopping=true;}wake.notify_all();thread.join();}
        std::ofstream f(path+".status.json");f<<"{\"drained\":"<<((!count&&!failed&&!overflow&&!rejected)?"true":"false")
            <<",\"failed\":"<<((failed||overflow||rejected)?"true":"false")<<",\"overflow\":"<<overflow<<",\"submitted\":"<<submitted
            <<",\"written\":"<<written<<",\"rejected_after_failure\":"<<rejected<<",\"remaining\":"<<count<<",\"high_water\":"<<peak<<",\"capacity\":8192}\n";
    }
};
Log log_instance;
uint64_t readnum(const uint8_t* p,size_t n,bool le){uint64_t v=0;for(size_t i=0;i<n;++i)v|=uint64_t(p[i])<<(8*(le?i:n-1-i));return v;}
}
extern "C" void mooncar_dds_event(MooncarDdsEvent e) noexcept {
    auto& l=log_instance;if(l.path.empty())return;
    auto wall=std::chrono::steady_clock::now().time_since_epoch();
    {std::lock_guard<std::mutex> guard(l.mutex);if(l.stopping||l.failed){++l.rejected;return;}++l.submitted;
        if(l.count==l.queue.size()){++l.overflow;return;}l.queue[l.head]={e,uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(wall).count())};
        l.head=(l.head+1)%l.queue.size();++l.count;if(l.count>l.peak)l.peak=l.count;}
    l.wake.notify_one();
}
extern "C" void mooncar_dds_flush() noexcept {log_instance.finish();}
extern "C" void mooncar_dds_wire(const char* stage,const void* data,uint32_t size,const uint8_t* segment,uint64_t offset,uint32_t validity) noexcept {
    auto p=static_cast<const uint8_t*>(data);if(size<20||std::memcmp(p,"RTPS",4))return;
    uint8_t prefix[12];std::memcpy(prefix,p+8,12);
    for(size_t pos=20;pos+4<=size;){bool le=p[pos+1]&1;size_t n=readnum(p+pos+2,2,le);size_t end=n?pos+4+n:size;if(end>size)break;
        auto b=p+pos+4;size_t len=end-pos-4;
        if(p[pos]==0x0c&&len>=20)std::memcpy(prefix,b+8,12);
        if(p[pos]==0x16&&len>=32&&readnum(b+28,4,le)>500000){MooncarDdsEvent e;e.stage=stage;
            if(segment)std::memcpy(e.segment,segment,8);e.offset=offset;e.validity=validity;
            std::memcpy(e.writer,prefix,12);std::memcpy(e.writer+12,b+8,4);
            e.sequence=(readnum(b+12,4,le)<<32)|readnum(b+16,4,le);e.fragment=readnum(b+20,4,le);e.fragments=readnum(b+24,2,le);
            e.a=readnum(b+26,2,le);e.b=readnum(b+28,4,le);
            size_t payload=4+readnum(b+2,2,le);
            // First fragment carries CDR encapsulation then builtin_interfaces/Time.
            if(e.fragment==1&&!(p[pos+1]&2)&&payload+12<=len){bool cdrle=b[payload+1]&1;
                e.stamp_ns=int64_t(int32_t(readnum(b+payload+4,4,cdrle)))*1000000000LL+readnum(b+payload+8,4,cdrle);}
            mooncar_dds_event(e);
        }if(!n)break;pos=end;
    }
}
