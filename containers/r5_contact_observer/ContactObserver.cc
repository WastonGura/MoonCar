// Test-only ECS observation. No component creation or state mutation.
#include <array>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>

namespace mooncar {
class ContactObserver final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  struct Row {
    std::uint64_t iteration{0}, self{0}, other{0};
    std::int64_t time{0}, dt{0};
    int wheel{0}, state{-1}, contacts{0}, points{0}, contact{-1}, point{-1};
    bool extra{false};
    std::array<double,13> values;
    Row() { values.fill(std::numeric_limits<double>::quiet_NaN()); }
  };
  const std::array<std::string,4> names{"front_left","front_right","rear_left","rear_right"};
  std::array<gz::sim::Entity,4> collisions{};
  std::array<Row,8192> queue;
  std::size_t head{0}, tail{0}, size{0};
  std::uint64_t written{0}, selected{0};
  bool stop{false}, working{false}, failed{false}, running{false}, pausedFlushed{false};
  std::mutex mutex;
  std::condition_variable ready, drained;
  std::thread writer;
  std::ofstream file;
  std::array<char,1048576> buffer{};
  std::string output;
  void enqueue(const Row &row) {
    std::lock_guard<std::mutex> lock(mutex);
    if (failed || size==queue.size()) {
      failed=true;
      throw std::runtime_error("Contact observer writer failed or bounded queue overflowed");
    }
    queue[tail]=row;tail=(tail+1)%queue.size();++size;++selected;ready.notify_one();
  }
  void writeRows() {
    while (true) {
      Row row;
      {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock,[this]{return size || stop;});
        if (!size && stop) break;
        row=queue[head];head=(head+1)%queue.size();--size;working=true;
      }
      file << row.iteration << ',' << row.time << ',' << row.dt << ',' << row.wheel
           << ',' << row.self << ',' << row.other << ',' << row.state << ',' << row.contacts
           << ',' << row.points << ',' << row.contact << ',' << row.point << ',' << row.extra;
      for (double value:row.values) file << ',' << value;
      file << '\n';
      {
        std::lock_guard<std::mutex> lock(mutex);
        failed=failed || !file;working=false;++written;
        if (!size) drained.notify_all();
      }
    }
  }
 public:
  gz::sim::System::PriorityType ConfigurePriority() override {return 110;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm)) throw std::runtime_error("Contact observer requires model");
    output=sdf->Get<std::string>("output");
    if (std::ifstream(output).good()) throw std::runtime_error("Contact observer refuses overwrite");
    std::ofstream identities(output+".entities");
    for (std::size_t w=0;w<names.size();++w) {
      auto link=model.LinkByName(ecm,names[w]+"_wheel_link");
      auto children=ecm.ChildrenByComponents(link,gz::sim::components::Collision());
      if (link==gz::sim::kNullEntity || children.size()!=1)
        throw std::runtime_error("Contact observer ambiguous wheel collision");
      collisions[w]=children.front();
      identities << w << ',' << collisions[w] << ',' << gz::sim::scopedName(collisions[w],ecm,"::",false) << '\n';
    }
    identities.close();if (!identities) throw std::runtime_error("Contact identities write failed");
    file.rdbuf()->pubsetbuf(buffer.data(),buffer.size());file.open(output);
    if (!file) throw std::runtime_error("Contact output unavailable");
    file << std::setprecision(17)
         << "iteration,t_ns,dt_ns,wheel,self_collision,other_collision,state,contact_count,point_count,contact_index,point_index,extra_complete,x,y,z,nx,ny,nz,depth,f1x,f1y,f1z,f2x,f2y,f2z\n";
    writer=std::thread(&ContactObserver::writeRows,this);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &) override {
    if (!info.paused || !running || pausedFlushed) return;
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!drained.wait_for(lock,std::chrono::seconds(5),[this]{return !size && !working;}) || failed)
        throw std::runtime_error("Contact drain timed out or writer failed");
    }
    file.flush();if (!file) throw std::runtime_error("Contact flush failed");
    std::ofstream marker(output+".paused");
    marker << info.iterations << ',' << std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()
           << ',' << selected << ',' << written << '\n';
    marker.close();if (!marker) throw std::runtime_error("Contact pause marker write failed");
    pausedFlushed=true;
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || info.dt<=std::chrono::steady_clock::duration::zero()) return;
    running=true;pausedFlushed=false;
    for (std::size_t w=0;w<names.size();++w) {
      Row base;base.iteration=info.iterations;base.time=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      base.dt=std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();base.wheel=static_cast<int>(w);base.self=collisions[w];
      const auto *component=ecm.Component<gz::sim::components::ContactSensorData>(collisions[w]);
      if (!component) {enqueue(base);continue;} // Missing is unknown, never zero force.
      const auto &contacts=component->Data();base.contacts=contacts.contact_size();
      base.state=base.contacts ? 1 : 0;
      if (!base.contacts) {enqueue(base);continue;} // Explicit present, empty snapshot.
      for (int c=0;c<base.contacts;++c) {
        const auto &contact=contacts.contact(c);Row row=base;row.contact=c;
        row.self=contact.collision1().id();row.other=contact.collision2().id();row.points=contact.position_size();
        if (!row.points) {enqueue(row);continue;}
        for (int k=0;k<row.points;++k) {
          row.point=k;row.values.fill(std::numeric_limits<double>::quiet_NaN());row.extra=false;
          const auto &position=contact.position(k);
          row.values[0]=position.x();row.values[1]=position.y();row.values[2]=position.z();
          if (contact.normal_size()==row.points && contact.depth_size()==row.points && contact.wrench_size()==row.points) {
            const auto &normal=contact.normal(k);const auto &wrench=contact.wrench(k);
            if (wrench.has_body_1_wrench() && wrench.body_1_wrench().has_force() &&
                wrench.has_body_2_wrench() && wrench.body_2_wrench().has_force()) {
              const auto &one=wrench.body_1_wrench().force();const auto &two=wrench.body_2_wrench().force();
              row.extra=true;row.values[3]=normal.x();row.values[4]=normal.y();row.values[5]=normal.z();row.values[6]=contact.depth(k);
              row.values[7]=one.x();row.values[8]=one.y();row.values[9]=one.z();row.values[10]=two.x();row.values[11]=two.y();row.values[12]=two.z();
            }
          }
          enqueue(row);
        }
      }
    }
  }
  ~ContactObserver() override {
    {std::lock_guard<std::mutex> lock(mutex);stop=true;}ready.notify_all();
    if (writer.joinable()) writer.join();
    file.flush();
    if (!output.empty()) {
      std::ofstream status(output+".status");
      status << "selected,written,queued,failed\n" << selected << ',' << written << ',' << size << ',' << (failed || !file) << '\n';
    }
  }
};
}
GZ_ADD_PLUGIN(mooncar::ContactObserver,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemConfigurePriority,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
