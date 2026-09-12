// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/nodal_wall_mapped/Layout.h"
namespace wall_response_frozen {
namespace d = tlfea::contact::nodal_wall_device_detail;
namespace fe = tl::fea;
using namespace tlfea::contact;
using nodal_wall_mapped::Sidecar;
using Code = NodalWallDeviceStatus;
TL_SURFACE_HD inline bool Response(d::Storage& storage,Sidecar side,const fe::DeviceNodalKinematicsView& k) {
  side.summary->rate=0;
  for(std::size_t g=0;g<side.groups;++g) side.traces[g]=0;
  for(unsigned i=0;i<storage.model.node_count;++i) {
    const auto& node=storage.result.nodes[i];
    const auto upper=::fmax(node.stiffness.upper,node.stiffness.value);
    if(upper==0) continue;
    const auto root=side.roots[i];
    if(root==UINT32_MAX) {
      double rate=0;
      if(!mass_detail::UpperProduct(upper,side.inverse[i],&rate))
        return d::Fail(storage.control,Code::NonFiniteArithmetic,node.node);
      side.summary->rate=::fmax(side.summary->rate,rate);
    } else {
      if(root>=side.groups) return d::Fail(storage.control,Code::InvalidInput,node.node);
      const auto* x=k.position_xyz+3*node.node;
      RigidNormalResponse response;
      if(EvaluateRigidNormalResponse(side.bodies[root],{x[0],x[1],x[2]},{-1,0,0},response)!=Status::kOk ||
          AccumulateRigidContactTrace(upper,response,side.traces[root])!=Status::kOk)
        return d::Fail(storage.control,Code::NonFiniteArithmetic,node.node);
    }
  }
  for(std::size_t g=0;g<side.groups;++g) side.summary->rate=::fmax(side.summary->rate,side.traces[g]);
  storage.result.diagnostics.stiffness_rate_bound=side.summary->rate;
  return true;
}
TL_SURFACE_HD inline void CheckResponse(d::Storage* pointer,Sidecar side,fe::NodalAssemblyView view) {
  auto& storage=*pointer;
  if(storage.control.status==Code::Ok && Response(storage,side,view.accepted)) {
    double step=0,frequency=0;
    if(!mass_detail::Upper(::sqrt(side.summary->rate),&frequency) ||
        !mass_detail::UpperProduct(storage.model.config.owner.fixed_dt,frequency,&step) || step>=1.6)
      d::Fail(storage.control,Code::StepTooLarge);
  }
}
} // namespace wall_response_frozen
