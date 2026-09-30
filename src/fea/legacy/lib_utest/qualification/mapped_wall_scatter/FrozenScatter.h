// Exact complete serial definitions from 4853da8; test-only namespace and
// host/device annotation allow the same independent packet on CPU and CUDA.
#pragma once
#include "lib_src/collision/NodalWallContactStorage.h"
#include "lib_src/collision/nodal_wall_mapped/Layout.h"
#include "lib_src/solvers/NodalForceAssembly.h"
#include "lib_src/solvers/NodalCinRuntime.h"
namespace wall_scatter_frozen {
using namespace tlfea::contact;
using namespace tlfea::contact::nodal_wall_device_detail;
using tlfea::contact::nodal_wall_mapped::Sidecar;
namespace d=tlfea::contact::nodal_wall_device_detail;
namespace fe=tl::fea;
namespace fea=tl::fea;
TL_SURFACE_HD inline bool StageStiffness(d::Storage& storage,Sidecar side,const fe::NodalCinAssemblyView& cin) {
  for(unsigned i=0;i<storage.model.node_count;++i) {
    const auto node=storage.model.nodes[i].node;
    const double before=cin.translational_stiffness[node];
    const double value=before+storage.result.nodes[i].stiffness.value;
    if(!IsFinite(before) || before<0 || !IsFinite(value) || value<0)
      return d::Fail(storage.control,Code::AssemblyFailure,node);
    side.stiffness[i]=value;
  }
  return true;
}
TL_SURFACE_HD inline bool Scatter(Storage& s,const fea::NodalAssemblyView& v,bool publish=true) {
  const auto owner_nodes=s.model.config.owner.node_count;
  double* actual[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                   v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for (unsigned c=0;c<6;++c) for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node;
    s.staged_force[c*owner_nodes+n]=actual[c][n];
  }
  const fea::DeviceNodalForceView staged{s.staged_force,s.staged_force+owner_nodes,s.staged_force+2*owner_nodes,
      s.staged_force+3*owner_nodes,s.staged_force+4*owner_nodes,s.staged_force+5*owner_nodes,
      v.forces.node_count,v.forces.base_epoch};
  for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node; const std::size_t index=n;
    const tl::math::Vec3 force{s.result.nodes[i].force_world.x,0,0},couple{};
    Q4IntegralInterval sum;
    if (fea::AccumulateNodalForces<1>(&index,&force,&couple,staged)!=fea::NodalForceAssemblyStatus::Success ||
        !q4_bounds::Add({actual[0][n],actual[0][n]},{force.x,force.x},&sum) ||
        !Radius(s.staged_force[n],sum,&s.addition_error[i])) return Fail(s.control,Code::AssemblyFailure,n);
  }
  if(publish) for (unsigned c=0;c<6;++c) for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node; actual[c][n]=s.staged_force[c*owner_nodes+n];
  }
  return true; // Legacy timestep rows are deliberately untouched.
}
// Infallible copy after mapped force AND stiffness destinations passed.
TL_SURFACE_HD inline void PublishScatter(Storage& s,const fea::NodalAssemblyView& view) {
  const auto nodes=s.model.config.owner.node_count;
  double* actual[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
      view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  for(unsigned c=0;c<6;++c) for(unsigned i=0;i<s.model.node_count;++i) {
    const auto node=s.model.nodes[i].node;
    actual[c][node]=s.staged_force[c*nodes+node];
  }
}
} // namespace wall_scatter_frozen
