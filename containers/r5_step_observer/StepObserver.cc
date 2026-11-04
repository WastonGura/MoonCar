// Test-only observation. Never creates or changes simulation components.
#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

namespace mooncar {
class StepObserver final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  std::ofstream file;
  std::string outputPath;
  bool seenRunning{false};
  bool flushedPaused{false};
  std::array<char, 1048576> buffer{};
  gz::sim::Model model{gz::sim::kNullEntity};
  std::array<double,6> forceBeforeStep{};
  std::array<double,4> wheelVelocityBeforeStep{};
  gz::transport::Node transport;
  std::mutex commandMutex;
  std::array<double,6> latestCommand{};
  std::array<double,6> commandBeforeStep{};
  std::uint64_t commandSequence{0}, sequenceBeforeStep{0};
  double commandSeenSim{0};
  const std::array<std::string,6> names{"camera_pan_joint","camera_tilt_joint",
    "front_left_wheel_joint","front_right_wheel_joint","rear_left_wheel_joint","rear_right_wheel_joint"};
  template<class T> double value(gz::sim::Entity entity,
                                const gz::sim::EntityComponentManager &ecm) const {
    const auto *c=ecm.Component<T>(entity);
    return c && !c->Data().empty() ? c->Data()[0] : std::numeric_limits<double>::quiet_NaN();
  }
 public:
  ~StepObserver() override { transport.Unsubscribe("/cmd_vel"); file.close(); }
  gz::sim::System::PriorityType ConfigurePriority() override { return 100; }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    {
      std::lock_guard<std::mutex> lock(commandMutex);
      if (sequenceBeforeStep!=commandSequence)
        commandSeenSim=std::chrono::duration<double>(info.simTime).count();
      commandBeforeStep=latestCommand;
      sequenceBeforeStep=commandSequence;
    }
    if (info.paused) {
      if (seenRunning && !flushedPaused) {
        file.flush();
        if (!file) throw std::runtime_error("StepObserver flush failed");
        std::ofstream marker(outputPath+".paused");
        marker << info.iterations << ',' << std::chrono::duration<double>(info.simTime).count() << '\n';
        marker.close();
        if (!marker) throw std::runtime_error("StepObserver pause confirmation failed");
        flushedPaused=true;
      }
      return;
    }
    // Read after position controllers and before Physics::Update clears commands.
    // Priority 100 follows the existing priority-zero position controllers.
    for (std::size_t n=0;n<names.size();++n)
      forceBeforeStep[n]=value<gz::sim::components::JointForceCmd>(model.JointByName(ecm,names[n]),ecm);
    for (std::size_t n=2;n<names.size();++n)
      wheelVelocityBeforeStep[n-2]=value<gz::sim::components::JointVelocityCmd>(model.JointByName(ecm,names[n]),ecm);
  }
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
    if (!model.Valid(ecm)) throw std::runtime_error("StepObserver requires a model");
    const auto path=sdf->Get<std::string>("output");
    outputPath=path;
    if (std::ifstream(path).good()) throw std::runtime_error("StepObserver refuses to overwrite evidence");
    file.rdbuf()->pubsetbuf(buffer.data(),buffer.size());
    file.open(path);
    if (!file) throw std::runtime_error("StepObserver output unavailable");
    file << std::setprecision(17) << "iteration,t,dt,x,y,z,qw,qx,qy,qz";
    for (const auto &name:names) file << ',' << name << "_q," << name << "_v," << name << "_force_cmd";
    file << ",input_lx,input_ly,input_lz,input_ax,input_ay,input_az,input_sequence,input_seen_sim";
    for (std::size_t n=2;n<names.size();++n) file << ',' << names[n] << "_velocity_cmd";
    file << '\n';
    // Observe the Gazebo input topic; this subscriber never publishes commands.
    if (!transport.Subscribe<gz::msgs::Twist>("/cmd_vel", [this](const gz::msgs::Twist &msg) {
        std::lock_guard<std::mutex> lock(commandMutex);
        latestCommand={msg.linear().x(),msg.linear().y(),msg.linear().z(),
                       msg.angular().x(),msg.angular().y(),msg.angular().z()};
        ++commandSequence;
      })) throw std::runtime_error("StepObserver command subscription failed");
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || info.dt<=std::chrono::steady_clock::duration::zero()) return;
    seenRunning=true;flushedPaused=false;
    const auto pose=gz::sim::worldPose(model.Entity(),ecm);
    file << info.iterations << ',' << std::chrono::duration<double>(info.simTime).count()
         << ',' << std::chrono::duration<double>(info.dt).count()
         << ',' << pose.Pos().X() << ',' << pose.Pos().Y() << ',' << pose.Pos().Z()
         << ',' << pose.Rot().W() << ',' << pose.Rot().X() << ',' << pose.Rot().Y() << ',' << pose.Rot().Z();
    for (std::size_t n=0;n<names.size();++n) {
      const auto &name=names[n];
      const auto joint=model.JointByName(ecm,name);
      file << ',' << value<gz::sim::components::JointPosition>(joint,ecm)
           << ',' << value<gz::sim::components::JointVelocity>(joint,ecm)
           << ',' << forceBeforeStep[n];
    }
    for (double command:commandBeforeStep) file << ',' << command;
    file << ',' << sequenceBeforeStep << ',' << commandSeenSim;
    for (double command:wheelVelocityBeforeStep) file << ',' << command;
    file << '\n';
    if (!file) throw std::runtime_error("StepObserver write failed");
  }
};
}
GZ_ADD_PLUGIN(mooncar::StepObserver,gz::sim::System,
              gz::sim::ISystemConfigure,gz::sim::ISystemConfigurePriority,
              gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
