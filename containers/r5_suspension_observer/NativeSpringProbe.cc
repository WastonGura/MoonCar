// Diagnostic-only interposition: forward the exact original getter, then read
// native DART joint state/properties. Never writes physics state or commands.
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dlfcn.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>

namespace dart::dynamics {
const Eigen::Vector6d& BodyNode::getBodyForce() const {
  using Getter=const Eigen::Vector6d& (*)(const BodyNode*);
  static auto original=reinterpret_cast<Getter>(dlsym(RTLD_NEXT,"_ZNK4dart8dynamics8BodyNode12getBodyForceEv"));
  if(!original)throw std::runtime_error("Original DART getter unavailable");
  const auto &force=original(this);
  const auto *joint=getParentJoint();
  if(joint && (joint->getName()=="spring" || joint->getName().find("_suspension_joint")!=std::string::npos)) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> guard(mutex);
    static std::ofstream file([](){
      const char *path=std::getenv("MOONCAR_NATIVE_SPRING_CSV");
      if(!path || std::ifstream(path).good())throw std::runtime_error("Native spring evidence path invalid");
      return path;
    }());
    static unsigned long call=0;
    if(call==0)file<<"call,joint,q,v,a,velocity_change_native,force_native,constraint_impulse_native,k_native,c_native,ref_native,fx_body,fy_body,fz_body,tx_body,ty_body,tz_body,parent_x,parent_y,parent_z,parent_qw,parent_qx,parent_qy,parent_qz,child_x,child_y,child_z,child_qw,child_qx,child_qy,child_qz\n";
    file<<std::setprecision(17)<<++call<<','<<joint->getName()<<','<<joint->getPosition(0)
        <<','<<joint->getVelocity(0)<<','<<joint->getAcceleration(0)<<','<<joint->getVelocityChange(0)<<','<<joint->getForce(0)
        <<','<<joint->getConstraintImpulse(0)<<','<<joint->getSpringStiffness(0)<<','<<joint->getDampingCoefficient(0)
        <<','<<joint->getRestPosition(0)<<','<<force[3]<<','<<force[4]<<','<<force[5]
        <<','<<force[0]<<','<<force[1]<<','<<force[2];
    for(const auto *body:{getParentBodyNode(),this}) {
      if(!body)throw std::runtime_error("Native suspension parent missing");
      const auto &pose=body->getWorldTransform();const Eigen::Quaterniond rotation(pose.linear());
      file<<','<<pose.translation().x()<<','<<pose.translation().y()<<','<<pose.translation().z()
          <<','<<rotation.w()<<','<<rotation.x()<<','<<rotation.y()<<','<<rotation.z();
    }
    file<<'\n';
    if(call%1000==0)file.flush();
    if(!file)throw std::runtime_error("Native spring evidence write failed");
  }
  return force;
}
}
