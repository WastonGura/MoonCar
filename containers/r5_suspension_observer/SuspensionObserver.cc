// Diagnostic state requests only; no force, velocity or position commands.
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>
#include <limits>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
namespace mooncar {
class SuspensionObserver final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
  public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  std::ofstream file;
  std::vector<gz::sim::Entity> joints;
  std::vector<gz::sim::Entity> parents,children,drives;
  std::vector<double> commands;
  gz::sim::Model model{gz::sim::kNullEntity};
  static void pose(std::ofstream &out,const gz::math::Pose3d &p) {
    out<<','<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()
       <<','<<p.Rot().W()<<','<<p.Rot().X()<<','<<p.Rot().Y()<<','<<p.Rot().Z();
  }
  template<class T> static double command(gz::sim::Entity entity,
      const gz::sim::EntityComponentManager &ecm) {
    const auto *c=ecm.Component<T>(entity);
    return c && !c->Data().empty() ? c->Data()[0] : std::numeric_limits<double>::quiet_NaN();
  }
 public:
  gz::sim::System::PriorityType ConfigurePriority() override {return 100;}
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(info.paused)return;
    commands.clear();
    for(std::size_t i=0;i<joints.size();++i) {
      commands.push_back(command<gz::sim::components::JointForceCmd>(joints[i],ecm));
      commands.push_back(command<gz::sim::components::JointVelocityCmd>(joints[i],ecm));
      commands.push_back(command<gz::sim::components::JointVelocityCmd>(drives[i],ecm));
    }
  }
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    const auto output=sdf->Get<std::string>("output");
    if (std::ifstream(output).good()) throw std::runtime_error("Refuse evidence overwrite");
    file.open(output); if (!file) throw std::runtime_error("Cannot open suspension evidence");
    file<<std::setprecision(17)<<"iteration,t,dt";
    model=gz::sim::Model(entity);
    const auto specification=sdf->Clone();
    auto element=specification->GetElement("joint");
    while(element) {
      const auto name=element->Get<std::string>();
      const auto joint=model.JointByName(ecm,name);
      if(joint==gz::sim::kNullEntity) throw std::runtime_error("Missing suspension joint: "+name);
      joints.push_back(joint);
      const gz::sim::Joint wrapper(joint);
      parents.push_back(model.LinkByName(ecm,wrapper.ParentLinkName(ecm).value()));
      children.push_back(model.LinkByName(ecm,wrapper.ChildLinkName(ecm).value()));
      gz::sim::Link(parents.back()).EnableVelocityChecks(ecm);
      const auto suffix=name.find("_suspension_joint");
      drives.push_back(suffix==std::string::npos ? gz::sim::kNullEntity :
          model.JointByName(ecm,name.substr(0,suffix)+"_joint"));
      wrapper.EnableTransmittedWrenchCheck(ecm);
      if(!ecm.Component<gz::sim::components::JointPosition>(joint))
        ecm.CreateComponent(joint,gz::sim::components::JointPosition());
      if(!ecm.Component<gz::sim::components::JointVelocity>(joint))
        ecm.CreateComponent(joint,gz::sim::components::JointVelocity());
      file<<','<<name<<"_q,"<<name<<"_v";
      for(const auto &field:{"vx","vy","vz"})file<<','<<name<<"_parent_world_"<<field;
      for(const auto &part:{"parent","child"})
        for(const auto &field:{"x","y","z","qw","qx","qy","qz"})file<<','<<name<<'_'<<part<<'_'<<field;
      for(const auto &field:{"fx","fy","fz","tx","ty","tz","k_imported","c_imported","ref_imported",
          "suspension_force_cmd","suspension_velocity_cmd","drive_velocity_cmd"})file<<','<<name<<'_'<<field;
      element=element->GetNextElement("joint");
    }
    file<<'\n';
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) {file.flush();return;}
    if(info.dt<=std::chrono::steady_clock::duration::zero()) return;
    file<<info.iterations<<','<<std::chrono::duration<double>(info.simTime).count()
        <<','<<std::chrono::duration<double>(info.dt).count();
    for(std::size_t i=0;i<joints.size();++i) {
      const auto joint=joints[i];
      const auto *q=ecm.Component<gz::sim::components::JointPosition>(joint);
      const auto *v=ecm.Component<gz::sim::components::JointVelocity>(joint);
      if(!q || !v || q->Data().size()!=1 || v->Data().size()!=1)
        throw std::runtime_error("Missing physical suspension state");
      file<<','<<q->Data()[0]<<','<<v->Data()[0];
      const auto velocity=gz::sim::Link(parents[i]).WorldLinearVelocity(ecm);
      if(!velocity)throw std::runtime_error("Missing physical parent velocity");
      file<<','<<velocity->X()<<','<<velocity->Y()<<','<<velocity->Z();
      pose(file,gz::sim::worldPose(parents[i],ecm));pose(file,gz::sim::worldPose(children[i],ecm));
      const gz::sim::Joint wrapper(joint);
      const auto wrench=wrapper.TransmittedWrench(ecm);
      if(!wrench || wrench->size()!=1)throw std::runtime_error("Missing native joint wrench");
      const auto &w=(*wrench)[0];
      file<<','<<w.force().x()<<','<<w.force().y()<<','<<w.force().z()
          <<','<<w.torque().x()<<','<<w.torque().y()<<','<<w.torque().z();
      const auto axis=wrapper.Axis(ecm);
      if(!axis || axis->size()!=1)throw std::runtime_error("Missing imported spring parameters");
      file<<','<<(*axis)[0].SpringStiffness()<<','<<(*axis)[0].Damping()<<','<<(*axis)[0].SpringReference();
      for(std::size_t j=0;j<3;++j)file<<','<<commands.at(3*i+j);
    }
    file<<'\n'; if(!file) throw std::runtime_error("Suspension evidence write failed");
  }
};
}
GZ_ADD_PLUGIN(mooncar::SuspensionObserver,gz::sim::System,
  gz::sim::ISystemConfigure,gz::sim::ISystemConfigurePriority,
  gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
