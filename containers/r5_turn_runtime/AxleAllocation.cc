// Scoped R5/v2 actuator; optional CSV is for acceptance observations.
// Receives only the original DiffDrive's limited joint commands, never truth.
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

namespace mooncar {
class AxleAllocation final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemPreUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  std::ofstream file;
  std::array<char,1048576> buffer{};
  std::string path;
  bool enabled{false},seenRunning{false},flushed{false};
  double front{1.0},rear{1.0};
  const std::array<std::string,4> names{"front_left","front_right","rear_left","rear_right"};
 public:
  ~AxleAllocation() override {file.close();}
  gz::sim::System::PriorityType ConfigurePriority() override {return 50;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
    if (!model.Valid(ecm)) throw std::runtime_error("AxleAllocation requires model");
    enabled=sdf->Get<bool>("enabled");front=sdf->Get<double>("front_factor");rear=sdf->Get<double>("rear_factor");
    if (!std::isfinite(front) || !std::isfinite(rear) || std::abs(front+rear-2)>1e-12 ||
        std::abs(front-1)>.05 || std::abs(rear-1)>.05)
      throw std::runtime_error("Invalid or out-of-scope allocation");
    path=sdf->HasElement("output") ? sdf->Get<std::string>("output") : "";
    if (path.empty()) return;
    if (path=="UNSET" || std::ifstream(path).good()) throw std::runtime_error("Unset/occupied controller evidence path");
    file.rdbuf()->pubsetbuf(buffer.data(),buffer.size());file.open(path);
    if (!file) throw std::runtime_error("Controller evidence unavailable");
    file << std::setprecision(17) << "iteration,t,dt,enabled,applied";
    for (const auto &n:names) file << ',' << n << "_native";
    for (const auto &n:names) file << ',' << n << "_allocated";
    file << '\n';
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) {
      if (file.is_open() && seenRunning && !flushed) {
        file.flush();if (!file) throw std::runtime_error("Controller flush failed");
        std::ofstream marker(path+".paused");marker << std::setprecision(17) << info.iterations << ','
          << std::chrono::duration<double>(info.simTime).count() << '\n';marker.close();
        if (!marker) throw std::runtime_error("Controller flush receipt failed");
        flushed=true;
      }
      return;
    }
    if (info.dt<=std::chrono::steady_clock::duration::zero()) return;
    std::array<gz::sim::Entity,4> joints{};std::array<double,4> original{},allocated{};
    for (std::size_t i=0;i<4;++i) {
      joints[i]=model.JointByName(ecm,names[i]+"_wheel_joint");
      const auto *cmd=ecm.Component<gz::sim::components::JointVelocityCmd>(joints[i]);
      if (!cmd || cmd->Data().size()!=1 || !std::isfinite(cmd->Data()[0]))
        throw std::runtime_error("Original DiffDrive target missing");
      original[i]=cmd->Data()[0];allocated[i]=original[i];
    }
    if (std::abs(original[0]-original[2])>1e-12 || std::abs(original[1]-original[3])>1e-12)
      throw std::runtime_error("Unexpected native drive allocation/order");
    const bool pureTurn=std::abs(original[0]+original[1])<=1e-12 && std::abs(original[0]-original[1])>1e-12;
    const bool apply=enabled && pureTurn;
    if (apply) {
      for (std::size_t i=0;i<4;++i) {
        allocated[i]=original[i]*(i<2?front:rear);
        ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joints[i],{allocated[i]});
      }
    }
    seenRunning=true;flushed=false;
    if (file.is_open()) {
    file << info.iterations << ',' << std::chrono::duration<double>(info.simTime).count() << ','
         << std::chrono::duration<double>(info.dt).count() << ',' << enabled << ',' << apply;
    for (double v:original) file << ',' << v;
    for (double v:allocated) file << ',' << v;
    file << '\n';if (!file) throw std::runtime_error("Controller evidence write failed");
    }
  }
};
}
GZ_ADD_PLUGIN(mooncar::AxleAllocation,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemConfigurePriority,gz::sim::ISystemPreUpdate)
