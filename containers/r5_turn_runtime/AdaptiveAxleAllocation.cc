// Experimental measured-motion feedback only. No body truth/contact component reads.
#include "GatePolicy.hh"
#include "FeedbackLease.hh"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

namespace mooncar {
class AdaptiveAxleAllocation final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemPreUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::transport::Node node;
  std::mutex mutex;
  FeedbackLease feedback;
  double appliedFactor{1.};
  bool seen{false},flushed{false};
  std::ofstream file;
  std::string output;
  const std::array<std::string,4> names{"front_left","front_right","rear_left","rear_right"};
 public:
  gz::sim::System::PriorityType ConfigurePriority() override {return 50;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
    if (!model.Valid(ecm)) throw std::runtime_error("Adaptive allocation requires R5 model");
    if (!node.Subscribe("/mooncar/turn_allocation",&AdaptiveAxleAllocation::Receive,this))
      throw std::runtime_error("Adaptive feedback subscription failed");
    output=sdf->HasElement("output")?sdf->Get<std::string>("output"):"";
    if (output.empty()) return;
    if (output=="UNSET" || std::ifstream(output).good()) throw std::runtime_error("Occupied/unset adaptive evidence output");
    file.open(output);if (!file) throw std::runtime_error("Adaptive evidence unavailable");
    file << std::setprecision(17) << "iteration,t,dt,feedback_sequence,feedback_stamp_ns,front_factor,rear_factor,pure_turn,fresh,inhibited";
    for (const auto &n:names) file << ',' << n << "_native";
    for (const auto &n:names) file << ',' << n << "_allocated";
    file << '\n';
  }
  void Receive(const gz::msgs::StringMsg &message) {
    std::lock_guard<std::mutex> lock(mutex);
    feedback.Receive(message.data());
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) {
      if (file.is_open() && seen && !flushed) {
        file.flush();if (!file) throw std::runtime_error("Adaptive evidence flush failed");
        std::ofstream mark(output+".paused");
        mark << std::setprecision(17) << info.iterations << ',' << std::chrono::duration<double>(info.simTime).count() << '\n';
        mark.close();if (!mark) throw std::runtime_error("Adaptive flush receipt failed");flushed=true;
      }
      return;
    }
    if (info.dt<=std::chrono::steady_clock::duration::zero()) return;
    seen=true;flushed=false;
    const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    std::array<gz::sim::Entity,4> joints{};std::array<double,4> original{},allocated{};
    for (std::size_t i=0;i<4;++i) {
      joints[i]=model.JointByName(ecm,names[i]+"_wheel_joint");
      const auto *cmd=ecm.Component<gz::sim::components::JointVelocityCmd>(joints[i]);
      if (!cmd || cmd->Data().size()!=1 || !std::isfinite(cmd->Data()[0]))
        throw std::runtime_error("Original limited drive target missing");
      original[i]=cmd->Data()[0];allocated[i]=original[i];
    }
    if (std::abs(original[0]-original[2])>1e-12 || std::abs(original[1]-original[3])>1e-12)
      throw std::runtime_error("Unexpected native drive ordering");
    FeedbackLease snapshot;
    {std::lock_guard<std::mutex> lock(mutex);snapshot=feedback;}
    const bool pure=PureTurn(original);
    const bool fresh=snapshot.Fresh(now);
    const double limit=.02*std::chrono::duration<double>(info.dt).count();
    if (pure && fresh) appliedFactor+=std::max(-limit,std::min(limit,snapshot.factor-appliedFactor));
    else appliedFactor=1.;
    for (std::size_t i=0;i<4;++i) {
      if (pure) {
        allocated[i]=fresh?original[i]*(i<2?appliedFactor:2-appliedFactor):0.;
        ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joints[i],{allocated[i]});
      }
    }
    if (file.is_open()) {
      file << info.iterations << ',' << std::chrono::duration<double>(info.simTime).count() << ','
           << std::chrono::duration<double>(info.dt).count() << ',' << snapshot.sequence << ',' << snapshot.stamp << ','
           << appliedFactor << ',' << 2-appliedFactor << ',' << pure << ',' << fresh << ',' << (pure && !fresh);
      for (double v:original) file << ',' << v;
      for (double v:allocated) file << ',' << v;
      file << '\n';if (!file) throw std::runtime_error("Adaptive evidence write failed");
    }
  }
};
}
GZ_ADD_PLUGIN(mooncar::AdaptiveAxleAllocation,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemConfigurePriority,gz::sim::ISystemPreUpdate)
