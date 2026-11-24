// Standalone DART constitutive check using the same secant law as the plugin.
#include "Law.hh"
#include <array>
#include <dart/simulation/World.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/TranslationalJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

struct Case {std::string name;mooncar::IntegralLaw law;double beta,sign,dt,rolling;bool fromRest;};
void run(const Case &spec,const std::filesystem::path &output) {
  using namespace dart;
  const auto world=simulation::World::create();world->setTimeStep(spec.dt);world->setGravity(Eigen::Vector3d(0,0,-1.62));
  world->getConstraintSolver()->setCollisionDetector(collision::OdeCollisionDetector::create());
  world->getConstraintSolver()->getCollisionOption().maxNumContacts=4;
  const auto ground=dynamics::Skeleton::create("ground");
  auto [fixed,floor]=ground->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation().z()=-.05;fixed->setTransformFromParentBodyNode(pose);
  auto surface=floor->createShapeNodeWith<dynamics::CollisionAspect,dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(20,20,.1)));
  surface->getDynamicsAspect()->setPrimaryFrictionCoeff(1);surface->getDynamicsAspect()->setSecondaryFrictionCoeff(1);
  surface->getDynamicsAspect()->setFirstFrictionDirection(Eigen::Vector3d::Zero());
  ground->setMobile(false);world->addSkeleton(ground);
  const auto slider=dynamics::Skeleton::create("slider");
  auto [joint,body]=slider->createJointAndBodyNodePair<dynamics::TranslationalJoint>(nullptr);
  const double mass=spec.law.normal/1.62;
  const auto shape=std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(.2,.2,.1));
  auto contact=body->createShapeNodeWith<dynamics::CollisionAspect,dynamics::DynamicsAspect>(shape);
  dynamics::Inertia inertia;inertia.setMass(mass);inertia.setMoment(shape->computeInertia(mass));body->setInertia(inertia);
  auto aspect=contact->getDynamicsAspect();aspect->setPrimaryFrictionCoeff(spec.law.mu);aspect->setSecondaryFrictionCoeff(.85);
  aspect->setFirstFrictionDirection(Eigen::Vector3d::UnitY());aspect->setSecondarySlipCompliance(0);
  joint->setPositions(Eigen::Vector3d(0,0,.052));world->addSkeleton(slider);
  const double lambda=std::tan(spec.beta*std::acos(-1.)/180);
  const double target=spec.rolling>0 ? spec.sign*spec.rolling*lambda : spec.sign*.02;
  const double q=spec.rolling>0 ? spec.law.Ratio(lambda) : (spec.beta>0 ? spec.law.mu : 0.);
  std::ofstream file(output/(spec.name+".csv"));if(!file)throw std::runtime_error("Bench output unavailable");
  file<<std::setprecision(17)<<"iteration,t,dt,phase,y,pre_vy,post_vy,rolling,c_native,fy,fz,contacts,target_vy,target_ratio,normal\n";
  const auto settle=static_cast<std::size_t>(std::llround(1/spec.dt));
  const auto total=static_cast<std::size_t>(std::llround(7/spec.dt));
  for(std::size_t i=0;i<total;++i){
    if(i==settle){world->setGravity(Eigen::Vector3d(0,spec.sign*q*1.62,-1.62));
      if(!spec.fromRest)joint->setVelocities(Eigen::Vector3d(0,spec.beta>0 ? target : 0.,0));}
    const auto pre=joint->getVelocities();
    const double coefficient=spec.law.Compliance(pre.y(),spec.rolling);
    aspect->setPrimarySlipCompliance(coefficient);world->step();
    const auto v=joint->getVelocities();Eigen::Vector3d force=Eigen::Vector3d::Zero();std::size_t count=0;
    const auto &collisions=world->getConstraintSolver()->getLastCollisionResult();
    for(std::size_t j=0;j<collisions.getNumContacts();++j){const auto &c=collisions.getContact(j);
      const auto *one=c.collisionObject1->getShapeFrame()->asShapeNode()->getBodyNodePtr().get();
      const auto *two=c.collisionObject2->getShapeFrame()->asShapeNode()->getBodyNodePtr().get();
      if(one==body || two==body){force+=one==body ? c.force : -c.force;++count;}}
    if(!v.allFinite() || !force.allFinite())throw std::runtime_error("Nonfinite native contact state");
    file<<i+1<<','<<world->getTime()<<','<<spec.dt<<','<<(i>=settle)<<','<<joint->getPositions().y()<<','<<pre.y()<<','<<v.y()<<','<<spec.rolling<<','<<coefficient<<','<<force.y()<<','<<force.z()<<','<<count<<','<<(spec.beta>0 ? target : 0.)<<','<<spec.sign*q<<','<<spec.law.normal<<'\n';
  }
  file.flush();file.close();if(file.fail())throw std::runtime_error("Bench close failed");
  std::cout<<"DONE "<<spec.name<<std::endl;
}
int main(int argc,char **argv){
  if(argc!=4)throw std::runtime_error("Expected output and frozen front/rear law files");
  const std::filesystem::path output=argv[1];std::filesystem::create_directory(output);
  if(std::filesystem::exists(output/"cases.csv"))throw std::runtime_error("Preserve native bench evidence");
  {std::ifstream maps("/proc/self/maps");std::ofstream saved(output/"native-maps.txt");saved<<maps.rdbuf();}
  std::array<mooncar::IntegralLaw,2> laws{};
  for(std::size_t k=0;k<2;++k)laws[k]=mooncar::IntegralLaw::Load(argv[2+k]);
  if(laws[0].Compliance(0,0)!=0 || std::abs(laws[0].Compliance(.02,0)-.02/(laws[0].normal*laws[0].mu))>1e-14)
    throw std::runtime_error("Zero-spin continuous limits failed");
  std::vector<Case> cases;
  for(std::size_t k=0;k<2;++k)for(const double beta:{.5,3.,10.,30.,45.,60.,80.,85.,87.5,88.})
    cases.push_back({(k==0 ? "front-" : "rear-")+std::to_string(beta),laws[k],beta,1,.00025,.03,false});
  cases.push_back({"front-reverse-45",laws[0],45,-1,.00025,.03,false});
  cases.push_back({"front-half-step-45",laws[0],45,1,.000125,.03,false});
  cases.push_back({"front-zero-spin",laws[0],45,1,.00025,0,false});
  cases.push_back({"front-zero-input",laws[0],0,1,.00025,0,false});
  cases.push_back({"front-from-rest-65",laws[0],65,1,.00025,.03,true});
  cases.push_back({"front-from-rest-half-step-65",laws[0],65,1,.000125,.03,true});
  std::ofstream ledger(output/"cases.csv");ledger<<"name,beta,sign,dt,rolling,from_rest\n";
  for(const auto &c:cases)ledger<<c.name<<','<<c.beta<<','<<c.sign<<','<<c.dt<<','<<c.rolling<<','<<c.fromRest<<'\n';
  ledger.close();for(const auto &c:cases)run(c,output);
}
