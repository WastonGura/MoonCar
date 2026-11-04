// Research contact law. Reads wheel-local physical velocity; never drives a joint.
#include "Law.hh"
#include "../r5_turn_runtime/GatePolicy.hh"
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/SlipComplianceCmd.hh>
namespace mooncar {
class NonlinearContact final:public gz::sim::System,public gz::sim::ISystemConfigure,
  public gz::sim::ISystemConfigurePriority,public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  std::array<std::string,4> names{"front_left","front_right","rear_left","rear_right"};
  std::array<LateralLaw,4> laws{};
  std::array<gz::sim::Entity,4> links{},joints{},collisions{};
  std::ofstream file,pauseFile;std::string output,mode;std::string pending;
  std::array<char,1048576> buffer{};bool running{false},flushed{false};
  std::array<gz::math::Vector3d,4> sourceVelocity{};
  std::array<gz::math::Vector3d,4> sourceAxis{};
  bool sourceReady{false},sourcePaused{true};
  std::uint64_t sourceIteration{0},pauseSequence{0},sourcePauseSequence{0};double sourceTime{0};
  const double radius=.076;
 public:
  gz::sim::System::PriorityType ConfigurePriority() override{return 70;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);mode=sdf->Get<std::string>("mode");output=sdf->Get<std::string>("output");
    if((mode!="observe" && mode!="nonlinear") || output=="UNSET" || std::ifstream(output).good())
      throw std::runtime_error("Invalid mode or occupied/unset nonlinear evidence");
    const auto configuration=sdf->Clone();
    for(std::size_t i=0;i<names.size();++i){
      links[i]=model.LinkByName(ecm,names[i]+"_wheel_link");joints[i]=model.JointByName(ecm,names[i]+"_wheel_joint");
      const auto found=ecm.ChildrenByComponents(links[i],gz::sim::components::Collision());
      if(found.size()!=1 || joints[i]==gz::sim::kNullEntity)throw std::runtime_error("Ambiguous nonlinear wheel");
      collisions[i]=found[0];gz::sim::Link(links[i]).EnableVelocityChecks(ecm);
      const auto p=configuration->GetElement(i<2 ? "front" : "rear");
      laws[i]={p->Get<double>("normal"),p->Get<double>("mu"),p->Get<double>("weight"),p->Get<double>("fast"),p->Get<double>("slow")};laws[i].Validate();
    }
    file.rdbuf()->pubsetbuf(buffer.data(),buffer.size());file.open(output);
    if(!file)throw std::runtime_error("Nonlinear evidence unavailable");
    {std::ifstream maps("/proc/self/maps");std::ofstream saved(output+".maps");saved<<maps.rdbuf();saved.close();
      if(!saved)throw std::runtime_error("Nonlinear actual-library map unavailable");}
    pauseFile.open(output+".paused-states.csv");
    if(!pauseFile)throw std::runtime_error("Paused native-state evidence unavailable");
    pauseFile<<std::setprecision(17)<<"snapshot,iteration,t";
    for(const auto &name:names)for(const auto &axis:{"vx","vy","vz","axis_x","axis_y","axis_z"})pauseFile<<','<<name<<'_'<<axis;
    pauseFile<<'\n';
    file<<std::setprecision(17)<<"iteration,t,dt,mode,pure_turn,ready,source_ready,source_paused,source_iteration,source_t,source_pause_snapshot";
    for(const auto &name:names)for(const auto &field:{"target","pre_vx","pre_vy","pre_vz","source_vx","source_vy","source_vz","axis_x","axis_y","axis_z","source_axis_x","source_axis_y","source_axis_z","lateral_speed","spin","rolling_speed","beta_proxy_deg","coefficient","before1","after1","before2","after2"})file<<','<<name<<'_'<<field;
    for(const auto &name:names)for(const auto &field:{"post_vx","post_vy","post_vz"})file<<','<<name<<'_'<<field;
    file<<'\n';
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(info.paused){
      if(running && !flushed){file.flush();pauseFile.flush();std::ofstream marker(output+".paused");
        marker<<std::setprecision(17)<<info.iterations<<','<<std::chrono::duration<double>(info.simTime).count()<<'\n';marker.close();
        if(!file || !marker)throw std::runtime_error("Nonlinear pause receipt failed");
        flushed=true;}
      return;
    }
    if(info.dt<=std::chrono::steady_clock::duration::zero())return;
    running=true;flushed=false;std::array<double,4> targets{};
    for(std::size_t i=0;i<4;++i){const auto *c=ecm.Component<gz::sim::components::JointVelocityCmd>(joints[i]);
      if(!c || c->Data().size()!=1)throw std::runtime_error("Missing original wheel command");
      targets[i]=c->Data()[0];}
    const bool pure=PureTurn(targets);std::ostringstream row;row<<std::setprecision(17);
    bool ready=true;
    for(std::size_t i=0;i<4;++i){const auto *spin=ecm.Component<gz::sim::components::JointVelocity>(joints[i]);
      const auto *c=ecm.Component<gz::sim::components::SlipComplianceCmd>(collisions[i]);
      ready=ready && gz::sim::Link(links[i]).WorldLinearVelocity(ecm).has_value() && spin && spin->Data().size()==1 && c && c->Data().size()==2;}
    row<<info.iterations<<','<<std::chrono::duration<double>(info.simTime).count()<<','<<std::chrono::duration<double>(info.dt).count()<<','<<(mode=="nonlinear")<<','<<pure<<','<<ready<<','<<sourceReady<<','<<sourcePaused<<','<<sourceIteration<<','<<sourceTime<<','<<sourcePauseSequence;
    if(!ready){
      if(std::chrono::duration<double>(info.simTime).count()>.01)throw std::runtime_error("Wheel state absent after startup allowance");
      for(std::size_t i=0;i<4*22;++i)row<<','<<std::numeric_limits<double>::quiet_NaN();
      pending=row.str();return; // Preserve native startup commands, never invent physical state.
    }
    for(std::size_t i=0;i<4;++i){
      const auto velocity=gz::sim::Link(links[i]).WorldLinearVelocity(ecm);
      const auto *spin=ecm.Component<gz::sim::components::JointVelocity>(joints[i]);
      auto *c=ecm.Component<gz::sim::components::SlipComplianceCmd>(collisions[i]);
      if(!velocity || !spin || spin->Data().size()!=1 || !c || c->Data().size()!=2)
        throw std::runtime_error("Missing local physical/contact state");
      const auto axis=gz::sim::worldPose(collisions[i],ecm).Rot().RotateVector(gz::math::Vector3d::UnitZ);
      const double lateral=velocity->Dot(axis),rolling=radius*std::abs(spin->Data()[0]);
      const double coefficient=laws[i].Compliance(lateral,rolling);
      auto after=c->Data();const double before1=after[0],before2=after[1];
      if(mode=="nonlinear"){after[0]=pure ? coefficient : 0.;ecm.SetComponentData<gz::sim::components::SlipComplianceCmd>(collisions[i],after);}
      row<<','<<targets[i]<<','<<velocity->X()<<','<<velocity->Y()<<','<<velocity->Z()<<','<<sourceVelocity[i].X()<<','<<sourceVelocity[i].Y()<<','<<sourceVelocity[i].Z()<<','<<axis.X()<<','<<axis.Y()<<','<<axis.Z()<<','<<sourceAxis[i].X()<<','<<sourceAxis[i].Y()<<','<<sourceAxis[i].Z()<<','<<lateral<<','<<spin->Data()[0]<<','<<rolling<<','<<std::atan2(std::abs(lateral),rolling)*180/std::acos(-1.)<<','<<coefficient<<','<<before1<<','<<after[0]<<','<<before2<<','<<after[1];
    }
    pending=row.str();
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
    sourceReady=true;
    for(std::size_t i=0;i<4;++i){const auto v=gz::sim::Link(links[i]).WorldLinearVelocity(ecm);
      if(!v){sourceReady=false;continue;}sourceVelocity[i]=*v;
      sourceAxis[i]=gz::sim::worldPose(collisions[i],ecm).Rot().RotateVector(gz::math::Vector3d::UnitZ);}
    sourcePaused=info.paused;sourceIteration=info.iterations;sourceTime=std::chrono::duration<double>(info.simTime).count();
    if(info.paused){
      if(!sourceReady)return;
      sourcePauseSequence=++pauseSequence;pauseFile<<sourcePauseSequence<<','<<sourceIteration<<','<<sourceTime;
      for(std::size_t i=0;i<4;++i)pauseFile<<','<<sourceVelocity[i].X()<<','<<sourceVelocity[i].Y()<<','<<sourceVelocity[i].Z()<<','<<sourceAxis[i].X()<<','<<sourceAxis[i].Y()<<','<<sourceAxis[i].Z();
      pauseFile<<'\n';pauseFile.flush();if(!pauseFile)throw std::runtime_error("Paused state flush failed");return;
    }
    sourcePauseSequence=0;
    if(info.dt<=std::chrono::steady_clock::duration::zero())return;
    if(!sourceReady)throw std::runtime_error("Missing poststep velocity");
    file<<pending;
    for(const auto &v:sourceVelocity)file<<','<<v.X()<<','<<v.Y()<<','<<v.Z();
    file<<'\n';if(!file)throw std::runtime_error("Nonlinear evidence write failed");
  }
};
}
GZ_ADD_PLUGIN(mooncar::NonlinearContact,gz::sim::System,gz::sim::ISystemConfigure,
  gz::sim::ISystemConfigurePriority,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
