// ODE API contract tests; no simulation, integration or constraint solve.
#include <ode/ode.h>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

static int reject(dGeomID, dGeomID, int) { return 0; }
static void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
int main() {
  dInitODE2(0);
  dReal vertices[]={-.25,0,0, 0,0,0, 0,.25,0, -.25,.25,0};
  dTriIndex indices[]={0,1,2,0,2,3};
  dTriMeshDataID data=dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildDouble(data,vertices,3*sizeof(dReal),4,indices,6,3*sizeof(dTriIndex));
  dGeomID mesh=dCreateTriMesh(0,data,nullptr,nullptr,nullptr);
  dGeomID wheel=dCreateCylinder(0,.076,.05);
  dQuaternion q={.64866268659553394,-.15418882279328777,.69009115841688629,.28149017570623031};
  dGeomSetQuaternion(wheel,q);
  dGeomSetPosition(wheel,-.22633955251347673,.029164878295443081,.075999998876827762);
  std::array<dContactGeom,4> output;
  std::memset(output.data(),0xa5,sizeof(output));
  dContactGeom guard=output[1];
  require(dCollide(wheel,mesh,1,output.data(),sizeof(dContactGeom))==1,"max contacts one");
  require(!std::memcmp(&guard,&output[1],sizeof(guard)),"contact output overrun");
  require(output[0].g1==wheel && output[0].g2==mesh,"geometry association");
  require(output[0].side1==-1 && output[0].side2>=0 && output[0].side2<2,"triangle association");
  require(output[0].depth>=0 && std::isfinite(output[0].depth),"valid penetration");
  dGeomTriMeshSetCallback(mesh,reject);
  require(dCollide(wheel,mesh,4,output.data(),sizeof(dContactGeom))==0,"triangle callback respected");
  dGeomTriMeshSetCallback(mesh,nullptr);
  require(dCollide(mesh,wheel,4,output.data(),sizeof(dContactGeom))>0,"reversed pair");
  require(output[0].g1==mesh && output[0].g2==wheel && output[0].normal[2]<0,"reversed normal");
  dGeomSetPosition(wheel,-.22633955251347673,.029164878295443081,.077);
  require(dCollide(wheel,mesh,4,output.data(),sizeof(dContactGeom))==0,"separation respected");
  std::cout<<"PASS: contact limit, buffer bound, geometry/triangle IDs, callback, reversed pair, separation\n";
  std::cout<<dGetConfiguration()<<"\n";
  dGeomDestroy(wheel);dGeomDestroy(mesh);dGeomTriMeshDataDestroy(data);dCloseODE();
}
