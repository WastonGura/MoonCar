// Drive-ablation-only copy of the existing idle lateral-slip policy.
// No wheel command is created: this preserves the active model's measured
// zero-input contact policy while removing DiffDrive and axle execution.
#include <array>
#include <fstream>
#include <stdexcept>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/SlipComplianceCmd.hh>
namespace mooncar {
class IdleSlipPolicy final: public gz::sim::System,public gz::sim::ISystemConfigure,
    public gz::sim::ISystemConfigurePriority,public gz::sim::ISystemPreUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  std::ofstream file;
  const std::array<std::string,4> names{"front_left","front_right","rear_left","rear_right"};
 public:
  gz::sim::System::PriorityType ConfigurePriority() override{return 60;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);const auto path=sdf->Get<std::string>("output");
    if(std::ifstream(path).good())throw std::runtime_error("Refuse idle-policy evidence overwrite");
    file.open(path);if(!file)throw std::runtime_error("Idle-policy evidence unavailable");
    file<<"iteration,wheel,lateral_before,lateral_after,longitudinal\n";
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(info.paused){file.flush();return;}
    for(std::size_t i=0;i<names.size();++i) {
      if(ecm.Component<gz::sim::components::JointVelocityCmd>(model.JointByName(ecm,names[i]+"_wheel_joint")))
        throw std::runtime_error("Drive ablation still has a wheel command");
      const auto collisions=ecm.ChildrenByComponents(model.LinkByName(ecm,names[i]+"_wheel_link"),gz::sim::components::Collision());
      if(collisions.size()!=1)throw std::runtime_error("Ambiguous ablation wheel collision");
      auto *c=ecm.Component<gz::sim::components::SlipComplianceCmd>(collisions.front());
      if(!c || c->Data().size()!=2)continue; // Native WheelSlip has not emitted yet.
      auto values=c->Data();const double before=values[0];values[0]=0;
      ecm.SetComponentData<gz::sim::components::SlipComplianceCmd>(collisions.front(),values);
      file<<info.iterations<<','<<i<<','<<before<<','<<values[0]<<','<<values[1]<<'\n';
    }
    if(!file)throw std::runtime_error("Idle-policy evidence write failed");
  }
};
}
GZ_ADD_PLUGIN(mooncar::IdleSlipPolicy,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemConfigurePriority,gz::sim::ISystemPreUpdate)
