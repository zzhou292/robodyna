// Complete frozen edab8e1 bodies; only qualification linkage/namespace changes.
#pragma once
#include "lib_src/collision/nodal_wall_mapped/Layout.h"
namespace wall_removal_frozen {
namespace m=tlfea::contact::nodal_wall_mapped;
namespace d=tlfea::contact::nodal_wall_device_detail;
namespace fe=tl::fea;
using Sidecar=m::Sidecar;
using Code=tlfea::contact::NodalWallDeviceStatus;
namespace nodal_wall_reduction=tlfea::contact::nodal_wall_reduction;
TL_SURFACE_HD inline bool RemovedPotential(d::Storage& storage,Sidecar side) {
  side.summary->removed_potential={};
  for(unsigned p=0;p<storage.model.parent_count;++p) {
    if(side.accepted[p]>1 || side.proposed[p]>side.accepted[p])
      return d::Fail(storage.control,Code::InvalidInput,UINT32_MAX,p);
    if(side.accepted[p] && !side.proposed[p] &&
        !nodal_wall_reduction::Sum(side.summary->removed_potential,storage.result.parents[p].potential))
      return d::Fail(storage.control,Code::NonFiniteArithmetic,UINT32_MAX,p);
  }
  return true;
}
TL_SURFACE_HD inline void FinishCandidate(d::Storage* pointer,m::Sidecar side,fe::NodalPreparedView view) {
  auto& storage=*pointer;
  if(storage.control.status==Code::Ok &&
      RemovedPotential(storage,side)) {
    storage.result.diagnostics.stiffness_rate_bound=side.summary->rate;
    storage.result.diagnostics.valid=true;
  }
}
}
