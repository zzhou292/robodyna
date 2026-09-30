#pragma once
#include "NodalWallOwnerFixture.h"
#include <vector>

namespace nodal_wall_capacity_test {
namespace sc=tlfea::contact;
constexpr unsigned Nodes=1030,Quads=804,Triangles=111,Parents=Quads+Triangles;
constexpr std::uint64_t FirstId=std::uint64_t{1}<<54;
constexpr long double SquareArea=1.L/16384;
// Synthetic 10x103 grid, three omitted interior quads and a triangle strip.
// All1030 nodes are incident. This is capacity evidence, not source Yaris.
struct Source {
  std::vector<double> x=std::vector<double>(3*Nodes);
  std::vector<sc::Q4ParametricReference> q=std::vector<sc::Q4ParametricReference>(Quads);
  std::vector<sc::T3MaterialMeasure> t=std::vector<sc::T3MaterialMeasure>(Triangles);
  std::vector<sc::NodalWallParentInput> input=std::vector<sc::NodalWallParentInput>(Parents);
  sc::VectorView positions() const { return {x.data(),Nodes,3,1}; }
  bool Prepare() {
    for(unsigned n=0;n<Nodes;++n) { x[3*n+1]=(static_cast<double>(n%10)-4.5)/128; x[3*n+2]=(static_cast<double>(n/10)-51)/128; }
    unsigned p=0;
    for(unsigned cell=0;cell<807;++cell) {
      if(cell==10 || cell==20 || cell==30) continue;
      const unsigned n=10*(cell/9)+cell%9; sc::SurfaceQ4 parent;
      parent.nodes[0]=n; parent.nodes[1]=n+1; parent.nodes[2]=n+11; parent.nodes[3]=n+10;
      parent.parent_element_id=FirstId+2*p+1; parent.feature_id=parent.parent_element_id+4096;
      if(q[p].Initialize(positions(),&parent,1).status!=sc::Q4ParametricStatus::Ok) return false;
      input[p]={&q[p],0,nullptr}; ++p;
    }
    for(p=0;p<Triangles;++p) {
      const unsigned cell=807+p,n=10*(cell/9)+cell%9; sc::SurfaceTriangle parent;
      parent.nodes[0]=n; parent.nodes[1]=cell%9?n+1:n+11; parent.nodes[2]=cell%9?n+11:n+10;
      parent.parent_element_id=FirstId+2*(Quads+p)+1; parent.feature_id=parent.parent_element_id+4096;
      parent.interpolation=sc::SurfaceInterpolation::kLinearTriangle;
      if(sc::PrepareT3MaterialMeasure(positions(),parent,&t[p])!=sc::SurfaceMeasureStatus::Ok) return false;
      input[Quads+p]={nullptr,0,&t[p]};
    }
    return true;
  }
};
} // namespace nodal_wall_capacity_test
