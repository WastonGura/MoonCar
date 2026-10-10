// Authorized test-only contact actuator. Does not read body truth or contacts.
#include "GatePolicy.hh"
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/SlipComplianceCmd.hh>
namespace mooncar {
class TurnSlipControl final : public gz::sim::System,
    public gz::sim::ISystemConfigure,public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemPreUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  const std::array<std::string,4> names{"front_left","front_right","rear_left","rear_right"};
  std::ofstream file;
  std::array<char,1048576> buffer{};
  std::string output,mode;
  bool seenRunning{false},flushed{false};
 public:
  gz::sim::System::PriorityType ConfigurePriority() override {return 60;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);mode=sdf->Get<std::string>("mode");output=sdf->Get<std::string>("output");
    if (!model.Valid(ecm) || (mode!="always" && mode!="pure_turn")) throw std::runtime_error("Invalid test slip-control mode/model");
    if (output=="UNSET" || std::ifstream(output).good()) throw std::runtime_error("Slip controller refuses unset/occupied output");
    file.rdbuf()->pubsetbuf(buffer.data(),buffer.size());file.open(output);
    if (!file) throw std::runtime_error("Slip controller output unavailable");
    file << std::setprecision(17) << "iteration,t,dt,pure_turn,control_mode";
    for (const auto &name:names) file << ',' << name << "_target," << name << "_present," << name << "_before1," << name << "_before2," << name << "_after1," << name << "_after2";
    file << '\n';
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) {
      if (seenRunning && !flushed) {
        file.flush();if (!file) throw std::runtime_error("Slip controller flush failed");
        std::ofstream marker(output+".paused");
        marker << std::setprecision(17) << info.iterations << ',' << std::chrono::duration<double>(info.simTime).count() << '\n';
        marker.close();if (!marker) throw std::runtime_error("Slip controller pause receipt failed");flushed=true;
      }
      return;
    }
    if (info.dt<=std::chrono::steady_clock::duration::zero()) return;
    seenRunning=true;flushed=false;
    std::array<double,4> targets{};
    for (std::size_t w=0;w<names.size();++w) {
      const auto *target=ecm.Component<gz::sim::components::JointVelocityCmd>(model.JointByName(ecm,names[w]+"_wheel_joint"));
      if (!target || target->Data().size()!=1) throw std::runtime_error("Missing original wheel target");
      targets[w]=target->Data()[0];
    }
    const bool pure=PureTurn(targets);
    file << info.iterations << ',' << std::chrono::duration<double>(info.simTime).count() << ','
         << std::chrono::duration<double>(info.dt).count() << ',' << pure << ',' << (mode=="pure_turn");
    for (std::size_t w=0;w<names.size();++w) {
      const auto link=model.LinkByName(ecm,names[w]+"_wheel_link");
      const auto collisions=ecm.ChildrenByComponents(link,gz::sim::components::Collision());
      if (collisions.size()!=1) throw std::runtime_error("Ambiguous wheel collision");
      auto *component=ecm.Component<gz::sim::components::SlipComplianceCmd>(collisions.front());
      const bool present=component && component->Data().size()==2;
      const double before1=present ? component->Data()[0] : std::numeric_limits<double>::quiet_NaN();
      const double before2=present ? component->Data()[1] : std::numeric_limits<double>::quiet_NaN();
      if (present && mode=="pure_turn" && !pure) {
        auto adjusted=component->Data();adjusted[0]=0.;
        ecm.SetComponentData<gz::sim::components::SlipComplianceCmd>(collisions.front(),adjusted);
      }
      // Keep the native longitudinal component untouched. Only the fixed
      // lateral command is suppressed outside pure turns.
      const double after1=present ? component->Data()[0] : std::numeric_limits<double>::quiet_NaN();
      const double after2=present ? component->Data()[1] : std::numeric_limits<double>::quiet_NaN();
      file << ',' << targets[w] << ',' << present << ',' << before1 << ',' << before2 << ',' << after1 << ',' << after2;
    }
    file << '\n';if (!file) throw std::runtime_error("Slip controller logging failed");
  }
  ~TurnSlipControl() override {file.close();}
};
}
GZ_ADD_PLUGIN(mooncar::TurnSlipControl,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemConfigurePriority,gz::sim::ISystemPreUpdate)
