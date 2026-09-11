// Frozen3eca05d ReduceNodes; namespace/callable qualifier only.
#pragma once
#include "lib_src/collision/NodalWallContactStorage.h"
namespace tlfea::contact::wall_observer_frozen {
using namespace nodal_wall_device_detail;
template<bool Physical=false>
TL_SURFACE_HD inline void ReduceNodes(Storage& s,const tl::fea::DeviceNodalKinematicsView& k) {
  const VectorView x{k.position_xyz,static_cast<std::uint32_t>(k.node_count),3,1};
  auto& d=s.result.diagnostics;
  if (s.control.status==Code::Ok) for (unsigned i=0;i<s.model.node_count;++i) {
    const auto& node=s.result.nodes[i];
    if (!nodal_wall_reduction::Sum(d.resultant,node.force) || !nodal_wall_reduction::Sum(d.potential,node.potential)) {
      Fail(s.control,Code::NonFiniteArithmetic,node.node); break;
    }
    d.wall_reaction=Add(d.wall_reaction,node.wall_reaction); d.wall_moment=Add(d.wall_moment,node.wall_moment);
    d.surface_power+=node.surface_power;
    const double penetration=x.at(node.node).x-s.model.config.law.wall_x;
    if ((!Physical || node.stiffness.value>0) && penetration>d.maximum_penetration) d.maximum_penetration=penetration;
    if (!IsFinite(d.wall_reaction) || !IsFinite(d.wall_moment) || !IsFinite(d.surface_power)) {
      Fail(s.control,Code::NonFiniteArithmetic,node.node); break;
    }
  }
  d.node_count=s.model.node_count; d.parent_count=s.model.parent_count; d.stiffness_rate_bound=s.model.rate;
}
}
