// Independent native DART constitutive-law test. No vehicle/control changes.
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
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

struct Case {
  std::string name;
  double c,mass,tangent,dt;
  std::size_t cap;
  bool orthogonal;
};

void run(const Case &spec,double normal,const std::filesystem::path &output) {
  using namespace dart;
  const auto world=simulation::World::create();
  world->setTimeStep(spec.dt);world->setGravity(Eigen::Vector3d(0,0,-normal));
  world->getConstraintSolver()->setCollisionDetector(collision::OdeCollisionDetector::create());
  world->getConstraintSolver()->getCollisionOption().maxNumContacts=spec.cap;
  const auto ground=dynamics::Skeleton::create("ground");
  auto [groundJoint,groundBody]=ground->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr);
  Eigen::Isometry3d groundPose=Eigen::Isometry3d::Identity();groundPose.translation().z()=-.05;
  groundJoint->setTransformFromParentBodyNode(groundPose);
  const auto groundShape=groundBody->createShapeNodeWith<dynamics::CollisionAspect,dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(10,10,.1)));
  groundShape->getDynamicsAspect()->setPrimaryFrictionCoeff(1);
  groundShape->getDynamicsAspect()->setSecondaryFrictionCoeff(1);
  groundShape->getDynamicsAspect()->setFirstFrictionDirection(Eigen::Vector3d::Zero());
  ground->setMobile(false);world->addSkeleton(ground);
  const auto slider=dynamics::Skeleton::create("slider");
  auto [joint,body]=slider->createJointAndBodyNodePair<dynamics::TranslationalJoint>(nullptr);
  body->setName("test_body");
  const auto box=std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(.2,.2,.1));
  const auto shape=body->createShapeNodeWith<dynamics::CollisionAspect,dynamics::DynamicsAspect>(box);
  dynamics::Inertia inertia;inertia.setMass(spec.mass);inertia.setMoment(box->computeInertia(spec.mass));
  body->setInertia(inertia);
  shape->getDynamicsAspect()->setPrimaryFrictionCoeff(.2);
  shape->getDynamicsAspect()->setSecondaryFrictionCoeff(.85);
  shape->getDynamicsAspect()->setPrimarySlipCompliance(spec.c);
  shape->getDynamicsAspect()->setSecondarySlipCompliance(0);
  shape->getDynamicsAspect()->setFirstFrictionDirection(Eigen::Vector3d::UnitY());
  joint->setPositions(Eigen::Vector3d(0,0,.052));world->addSkeleton(slider);
  const auto path=output/(spec.name+".csv");
  if(std::filesystem::exists(path))throw std::runtime_error("Preserve existing bench evidence");
  std::ofstream file(path);if(!file)throw std::runtime_error("Bench evidence unavailable");
  const auto pointsPath=output/(spec.name+".points.csv");
  if(std::filesystem::exists(pointsPath))throw std::runtime_error("Preserve contact evidence");
  std::ofstream points(pointsPath);if(!points)throw std::runtime_error("Contact evidence unavailable");
  points<<std::setprecision(17)<<"iteration,t,index,contacts,px,py,pz,nx,ny,nz,depth,fx,fy,fz,vx,vy,vz,c_native\n";
  file<<std::setprecision(17)<<"iteration,t,dt,phase,x,y,z,vx,vy,vz,fx,fy,fz,contacts,c_native,mass_native\n";
  const auto steps=static_cast<std::size_t>(std::llround(7/spec.dt));
  const auto settle=static_cast<std::size_t>(std::llround(1/spec.dt));
  for(std::size_t i=0;i<steps;++i) {
    if(i==settle)world->setGravity(spec.orthogonal ? Eigen::Vector3d(spec.tangent,0,-normal) :
        Eigen::Vector3d(0,spec.tangent,-normal));
    world->step();
    const auto q=joint->getPositions();const auto v=joint->getVelocities();
    if(!q.allFinite() || !v.allFinite())throw std::runtime_error("Nonfinite native state");
    Eigen::Vector3d force=Eigen::Vector3d::Zero();std::size_t count=0;
    const auto &result=world->getConstraintSolver()->getLastCollisionResult();
    for(std::size_t c=0;c<result.getNumContacts();++c) {
      const auto &point=result.getContact(c);
      const auto *one=point.collisionObject1->getShapeFrame()->asShapeNode()->getBodyNodePtr().get();
      const auto *two=point.collisionObject2->getShapeFrame()->asShapeNode()->getBodyNodePtr().get();
      if(one==body || two==body) {
        const Eigen::Vector3d wheelForce=one==body ? point.force : -point.force;
        force+=wheelForce;++count;
        points<<i+1<<','<<world->getTime()<<','<<c<<','<<result.getNumContacts();
        for(double value:point.point)points<<','<<value;
        for(double value:point.normal)points<<','<<value;
        points<<','<<point.penetrationDepth;
        for(double value:wheelForce)points<<','<<value;
        for(double value:v)points<<','<<value;
        points<<','<<shape->getDynamicsAspect()->getPrimarySlipCompliance()<<'\n';
      }
    }
    file<<i+1<<','<<world->getTime()<<','<<spec.dt<<','<<(i>=settle);
    for(double value:q)file<<','<<value;
    for(double value:v)file<<','<<value;
    for(double value:force)file<<','<<value;
    file<<','<<count<<','<<shape->getDynamicsAspect()->getPrimarySlipCompliance()<<','<<body->getMass()<<'\n';
    if(!file || !points)throw std::runtime_error("Bench evidence write failed");
  }
  file.flush();if(!file)throw std::runtime_error("Bench evidence flush failed");file.close();
  if(file.fail())throw std::runtime_error("Bench evidence close failed");
  points.flush();if(!points)throw std::runtime_error("Contact evidence flush failed");points.close();
  if(points.fail())throw std::runtime_error("Contact evidence close failed");
  std::cout<<"completed "<<spec.name<<std::endl;
}

int main(int argc,char **argv) {
  if(argc!=6)throw std::runtime_error("Expected output, compliance, tangent gravity, normal gravity, mass");
  const std::filesystem::path output=argv[1];
  const double c=std::stod(argv[2]),g=std::stod(argv[3]),normal=std::stod(argv[4]),mass=std::stod(argv[5]);
  if(!std::isfinite(c+g+normal+mass) || c<=0 || g<=0 || normal<=0 || mass<=0)
    throw std::runtime_error("Invalid bench declaration");
  const std::array<Case,10> cases{{
    {"01-nominal",c,mass,g,.00025,4,false},
    {"02-no-tangent-gravity",c,mass,0,.00025,4,false},
    {"03-reverse-gravity",c,mass,-g,.00025,4,false},
    {"04-zero-compliance",0,mass,g,.00025,4,false},
    {"05-half-compliance",c/2,mass,g,.00025,4,false},
    {"06-double-compliance",2*c,mass,g,.00025,4,false},
    {"07-half-step",c,mass,g,.000125,4,false},
    {"08-single-contact",c,mass,g,.00025,1,false},
    {"09-double-mass",c,2*mass,g,.00025,4,false},
    {"10-orthogonal-force",c,mass,g,.00025,4,true}
  }};
  std::filesystem::create_directories(output);
  const auto ledger=output/"cases.csv";
  if(std::filesystem::exists(ledger))throw std::runtime_error("Preserve prior declared cases");
  std::ofstream file(ledger);file<<std::setprecision(17)<<"name,c,mass,tangent,dt,cap,orthogonal\n";
  for(const auto &spec:cases)file<<spec.name<<','<<spec.c<<','<<spec.mass<<','<<spec.tangent<<','<<spec.dt<<','<<spec.cap<<','<<spec.orthogonal<<'\n';
  file.close();if(file.fail())throw std::runtime_error("Declaration close failed");
  for(const auto &spec:cases){std::cout<<"started "<<spec.name<<std::endl;run(spec,normal,output);}
}
