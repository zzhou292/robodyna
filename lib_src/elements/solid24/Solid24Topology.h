// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid24Types.h"
#include "lib_src/elements/solid6z/CollapsedBrickTopology.h"
#include "lib_src/math/ScalarBits.h"
namespace tl::fea::solid24::detail {
TL_BRICK_HD inline bool SamePosition(const Vec3& a,const Vec3& b)noexcept {
 return tl::math::SameScalarBits(a.x,b.x)&&tl::math::SameScalarBits(a.y,b.y)&&tl::math::SameScalarBits(a.z,b.z);
}
TL_BRICK_HD inline Status ValidateConnectivity(const ReferenceInput& input,unsigned& unique_count)noexcept {
 const auto profile=input.profile.connectivity;
 if(profile!=ConnectivityProfile::EightDistinct&&profile!=ConnectivityProfile::CollapsedTopEdges)return Status::UnsupportedProfile;
 for(unsigned n=0;n<8;++n)if(!input.source_node_id[n]||!solid_common::Finite(input.position_m[n]))return Status::InvalidInput;
 if(profile==ConnectivityProfile::CollapsedTopEdges){
  // Reuse only the authenticated ID-pattern recognizer. Discard its six-node
  // conversion map: all HEPH geometry, force, mass and assembly retain raw8.
  solid6z::CollapsedBrickTopology topology;
  const auto status=solid6z::MapCollapsedTopEdges(input.source_node_id,topology);
  if(status==solid6z::Status::InvalidInput)return Status::InvalidInput;
  if(status!=solid6z::Status::Success)return Status::UnsupportedProfile;
  if(!SamePosition(input.position_m[4],input.position_m[5])||!SamePosition(input.position_m[6],input.position_m[7]))return Status::InvalidInput;
  unique_count=6;return Status::Success;
 }
 for(unsigned n=0;n<8;++n)for(unsigned p=0;p<n;++p)
  if(input.source_node_id[n]==input.source_node_id[p])return Status::UnsupportedProfile;
 unique_count=8;return Status::Success;
}
TL_BRICK_HD inline bool ValidCurrentAliases(const ReferenceInput& input,
 const Vec3 (&position)[8],const Vec3 (&velocity)[8])noexcept {
 if(input.profile.connectivity==ConnectivityProfile::EightDistinct)return true;
 if(input.profile.connectivity!=ConnectivityProfile::CollapsedTopEdges)return false;
 return SamePosition(position[4],position[5])&&SamePosition(position[6],position[7])&&
        SamePosition(velocity[4],velocity[5])&&SamePosition(velocity[6],velocity[7]);
}
} // namespace tl::fea::solid24::detail
