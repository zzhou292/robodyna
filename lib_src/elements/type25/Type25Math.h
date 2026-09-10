// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete donor hashes: lib_utest/qualification/type25/native/source-manifest.json.
// GNU AGPL v3 or later; see LICENSE.md alongside.
#pragma once
#include "Type25Response.h"
#include "Type25Stability.h"

#if defined(__CUDACC__)
#define TL_TYPE25_HD __host__ __device__
#else
#define TL_TYPE25_HD
#endif

namespace tl::fea::type25 {
// Stateless value operation. Nodes must be the actual endpoint positions and
// their accepted interval midpoint velocity/spin. `dt` is that same interval.
// The caller authenticates phase/owner identity; this API creates no token or
// acceptance authority. All outputs are staged and unchanged on failure.
TL_TYPE25_HD inline Status Evaluate(SourceUnits units,const Property& property,const Reference& reference,
    const History& accepted,const EndpointKinematics (&nodes)[2],double dt,Evaluation& output) {
  detail::Units u;
  if(!detail::ResolveUnits(units,u)||!ValidProperty(property)||!detail::ValidHistory(accepted)||
     !detail::ValidReference(reference))return Status::InvalidInput;
  Evaluation next;next.history=accepted;
  auto status=AdvanceFrame(units,accepted.transverse_axis,nodes,dt,next.frame);
  if(status!=Status::Success)return status;
  status=detail::Deformation(u,reference,accepted,next.frame,nodes,dt,next.history);
  if(status!=Status::Success)return status;
  status=detail::Response(property,accepted,dt,next.history);
  if(status!=Status::Success)return status;
  // R4CUM3: force pair plus the two finite-length shear arms. The signs of
  // the endpoint moments are distinct; replacing this by +/- one couple
  // would violate total angular momentum for a transverse spring force.
  const auto f=next.history.local_force_N,m=next.history.local_couple_Nm;
  const double arm=.5*next.frame.length_m;
  const Vec3 m1{m.x,m.y-arm*f.z,m.z+arm*f.y},m2{m.x,m.y+arm*f.z,m.z-arm*f.y};
  next.endpoints[0]={tl::math::fixed3::ToWorld(next.frame.axes,f),tl::math::fixed3::ToWorld(next.frame.axes,m1)};
  next.endpoints[1]={tl::math::fixed3::Scale(next.endpoints[0].force_N,-1),
      tl::math::fixed3::Scale(tl::math::fixed3::ToWorld(next.frame.axes,m2),-1)};
  for(unsigned i=0;i<2;++i)if(!tl::math::fixed3::Finite(next.endpoints[i].force_N)||!tl::math::fixed3::Finite(next.endpoints[i].couple_Nm))
    return Status::NonfiniteResult;
  Stability stable;status=CriticalStep(units,property,next.frame.length_m,stable);
  if(status!=Status::Success)return status;
  next.critical_dt_s=stable.critical_dt_s;next.translation_stiffness_N_per_m=stable.translation_stiffness_N_per_m;
  next.rotation_stiffness_Nm_per_rad=stable.rotation_stiffness_Nm_per_rad;
  output=next;return Status::Success;
}
} // namespace tl::fea::type25

#undef TL_TYPE25_HD
