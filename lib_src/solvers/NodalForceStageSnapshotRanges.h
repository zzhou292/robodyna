#pragma once
#include "FENodalState.h"
#include "NodalTrialIdentity.h"

namespace tl::fea::nodal_detail {
inline NodalReport ValidateForceStageOutputs(NodalForceStageSnapshotBuffer out,std::size_t n,std::size_t g,
    NodalPreparedView* prepared,const NodalTrialToken& token) noexcept {
  if(!out.acceleration_xyz||!out.angular_acceleration_xyz||!out.groups||!prepared)
    return {NodalStatus::InvalidInput,"Missing force-stage snapshot output"};
  if(out.capacity_nodes<n||out.capacity_groups<g)
    return {NodalStatus::ResourceLimit,"Force-stage snapshot capacity is insufficient"};
  const void* ranges[]{out.acceleration_xyz,out.angular_acceleration_xyz,out.groups,prepared};
  const std::size_t bytes[]{3*n*sizeof(double),3*n*sizeof(double),g*sizeof(*out.groups),sizeof(*prepared)};
  for(unsigned i=0;i<4;++i) {
    if(!trial_identity::Disjoint(ranges[i],bytes[i],&token,sizeof(token)))
      return {NodalStatus::InvalidInput,"Force-stage output overlaps token or overflows"};
    for(unsigned j=0;j<i;++j)
      if(!trial_identity::Disjoint(ranges[i],bytes[i],ranges[j],bytes[j]))
        return {NodalStatus::InvalidInput,"Force-stage snapshot outputs overlap"};
  }
  return {NodalStatus::Ok,"OK"};
}
} // namespace tl::fea::nodal_detail
