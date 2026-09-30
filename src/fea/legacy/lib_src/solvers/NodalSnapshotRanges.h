#pragma once
#include "FENodalState.h"
#include "NodalTrialIdentity.h"
#include <cstring>

namespace tl::fea::nodal_detail {
// Shared host destination preflight and infallible publication for accepted and
// prepared snapshots. Owner startup already bounds n and all slab offsets.
struct SnapshotRanges {
  void* output[7];
  std::size_t bytes[7],offset[6];
  SnapshotRanges(NodalSnapshotBuffer out,std::size_t n,void* identity,std::size_t identity_bytes)
      :output{out.position_xyz,out.velocity_xyz,out.orientation_wxyz,out.angular_velocity_xyz,
              out.reaction_force_xyz,out.reaction_couple_xyz,identity},
       bytes{3*n*sizeof(double),3*n*sizeof(double),4*n*sizeof(double),3*n*sizeof(double),
             3*n*sizeof(double),3*n*sizeof(double),identity_bytes},
       offset{0,3*n,9*n,6*n,13*n,16*n} {}
  NodalReport Validate(NodalSnapshotBuffer out,std::size_t n,bool rotations,
                       const void* token=nullptr,std::size_t token_bytes=0) const noexcept {
    if(!output[6]||!out.position_xyz||!out.velocity_xyz)
      return {NodalStatus::InvalidInput,"Missing host snapshot output"};
    if(out.capacity_nodes<n) return {NodalStatus::ResourceLimit,"Snapshot capacity is insufficient"};
    if(!rotations&&(out.orientation_wxyz||out.angular_velocity_xyz||out.reaction_force_xyz||out.reaction_couple_xyz))
      return {NodalStatus::UnsupportedRotation,"Extended snapshot fields require extended nodal initialization"};
    for(unsigned i=0;i<7;++i) {
      if(!output[i]) continue;
      if(bytes[i]>UINTPTR_MAX-reinterpret_cast<std::uintptr_t>(output[i]))
        return {NodalStatus::InvalidInput,"Snapshot output address range overflows"};
      for(unsigned j=0;j<i;++j)
        if(output[j]&&!trial_identity::Disjoint(output[i],bytes[i],output[j],bytes[j]))
          return {NodalStatus::InvalidInput,"Snapshot outputs overlap"};
      if(token&&!trial_identity::Disjoint(output[i],bytes[i],token,token_bytes))
        return {NodalStatus::InvalidInput,"Snapshot output overlaps its trial token"};
    }
    return {NodalStatus::Ok,"OK"};
  }
  void PublishFields(const double* staged) const noexcept {
    for(unsigned i=0;i<6;++i)
      if(output[i]) std::memcpy(output[i],staged+offset[i],bytes[i]);
  }
};
} // namespace tl::fea::nodal_detail
