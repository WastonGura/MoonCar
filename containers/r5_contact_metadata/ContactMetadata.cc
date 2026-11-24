// Test-only identity snapshot. Never creates or changes ECS components.
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Physics.hh>
namespace mooncar {
class ContactMetadata final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate {
  gz::sim::Entity world{gz::sim::kNullEntity};
  std::ofstream stream;
  bool saved{false};
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    world=entity;
    const auto output=sdf->Get<std::string>("output");
    if (std::ifstream(output).good()) throw std::runtime_error("Metadata refuses overwrite");
    stream.open(output);
    if (!stream) throw std::runtime_error("Metadata output unavailable");
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override {
    if (saved || info.paused) return;
    const auto *detector=ecm.Component<gz::sim::components::PhysicsCollisionDetector>(world);
    if (!detector && info.iterations<100) return;
    stream << "iteration,t_ns,kind,entity,value\n" << info.iterations << ','
           << std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()
           << ",collision_detector," << world << ','
           << (detector ? detector->Data() : "MISSING") << '\n';
    ecm.Each<gz::sim::components::Collision, gz::sim::components::Name>(
      [&](const gz::sim::Entity &entity, const gz::sim::components::Collision *,
          const gz::sim::components::Name *) {
        const auto name=gz::sim::scopedName(entity,ecm,"::",false);
        if (name.find_first_of(",\r\n")!=std::string::npos)
          throw std::runtime_error("Unsupported collision name encoding");
        stream << info.iterations << ','
               << std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()
               << ",collision," << entity << ',' << name << '\n';
        return true;
      });
    stream.flush();
    if (!stream) throw std::runtime_error("Metadata write failed");
    saved=true;
  }
};
}
GZ_ADD_PLUGIN(mooncar::ContactMetadata,gz::sim::System,
              gz::sim::ISystemConfigure,gz::sim::ISystemPostUpdate)
