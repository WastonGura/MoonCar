// Actual gz-common OBJ loading, without Gazebo, dynamics or ROS initialization.
#include <cmath>
#include <iomanip>
#include <iostream>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
int main(int argc,char **argv) {
  if (argc<2) return 2;
  bool passed=true;
  std::cout << "{\"new_simulations\":0,\"meshes\":[";
  for (int a=1;a<argc;++a) {
    if (a>1) std::cout << ',';
    const auto *mesh=gz::common::MeshManager::Instance()->Load(argv[a]);
    bool valid=mesh && mesh->SubMeshCount()>0;
    unsigned int vertices=0,normals=0,indices=0;
    if (mesh) for (unsigned int i=0;i<mesh->SubMeshCount();++i) {
      const auto part=mesh->SubMeshByIndex(i).lock();
      if (!part) { valid=false;continue; }
      vertices+=part->VertexCount();normals+=part->NormalCount();indices+=part->IndexCount();
      valid=valid && part->VertexCount()>0 && part->NormalCount()==part->VertexCount()
        && part->SubMeshPrimitiveType()==gz::common::SubMesh::TRIANGLES && part->IndexCount()%3==0;
      for (unsigned int j=0;j<part->NormalCount();++j) {
        const auto n=part->Normal(j);
        valid=valid && std::isfinite(n.X()) && std::isfinite(n.Y()) && std::isfinite(n.Z())
          && std::abs(n.X())<1e-12 && std::abs(n.Y())<1e-12 && std::abs(n.Z()-1)<1e-12;
      }
      for (unsigned int j=0;j<part->IndexCount();++j) {
        const int index=part->Index(j);
        valid=valid && index>=0 && static_cast<unsigned int>(index)<part->VertexCount();
      }
    }
    passed=passed && valid;
    std::cout << "{\"path\":" << std::quoted(argv[a]) << ",\"vertices\":" << vertices
      << ",\"normals\":" << normals << ",\"indices\":" << indices << ",\"passed\":" << (valid?"true":"false") << '}';
  }
  std::cout << "],\"passed\":" << (passed?"true":"false") << "}\n";
  return passed?0:1;
}
