// SPDX-License-Identifier: AGPL-3.0-or-later
// PFINT3/PFCUM3/PMCUM3, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "ForceTypes.h"
#include "lib_src/solvers/NodalForceAssembly.h"

namespace tl::fea::beam18 {
namespace force_detail {
TL_BEAM18_HD inline Vec3 World(const ForceGeometry& g, Vec3 value) noexcept {
  const auto& a = g.axis;
  return {a[0].x*value.x+a[1].x*value.y+a[2].x*value.z,
          a[0].y*value.x+a[1].y*value.y+a[2].y*value.z,
          a[0].z*value.x+a[1].z*value.y+a[2].z*value.z};
}
TL_BEAM18_HD inline bool EndpointForces(ForceTrial& trial) noexcept {
  const auto f = trial.diagnostics.damped_section_force_n;
  const auto m = trial.diagnostics.damped_section_moment_nm;
  const auto length = trial.geometry.length_m;
  Vec3 local_force[2]{{-f.x,-f.y,-f.z},{}};
  local_force[1] = {-local_force[0].x,-local_force[0].y,-local_force[0].z};
  Vec3 local_moment[2]{{-m.x,-m.y+.5*length*f.z,-m.z-.5*length*f.y},
      {m.x,m.y+.5*length*f.z,m.z-.5*length*f.y}};
  // Native resolved IR1Y/IR2Y/IR1Z/IR2Z are all one, including zero additions.
  local_moment[0].y = 1.*local_moment[0].y+(1.-1.)*local_moment[1].y;
  local_moment[1].y = 1.*local_moment[1].y+(1.-1.)*local_moment[0].y;
  local_moment[0].z = 1.*local_moment[0].z+(1.-1.)*local_moment[1].z;
  local_moment[1].z = 1.*local_moment[1].z+(1.-1.)*local_moment[0].z;
  for (unsigned n = 0; n < 2; ++n) {
    const auto world_f = World(trial.geometry,local_force[n]);
    const auto world_m = World(trial.geometry,local_moment[n]);
    trial.rhs_force_n[n] = {-world_f.x,-world_f.y,-world_f.z};
    trial.rhs_couple_nm[n] = {-world_m.x,-world_m.y,-world_m.z};
    if (!tl::math::fixed3::Finite(world_f) || !tl::math::fixed3::Finite(world_m)) return false;
  }
  return true;
}
} // namespace force_detail
// Completed endpoint-only packet. Existing transactional owner is responsible
// for exact source/domain mapping, epoch, stiffness destinations and ordering.
TL_BEAM18_HD inline NodalForceAssemblyStatus AccumulateForces(const std::size_t (&nodes)[2],
    const ForceTrial& trial, DeviceNodalForceView destination) noexcept {
  return AccumulateNodalForces<2>(nodes,trial.rhs_force_n,trial.rhs_couple_nm,destination);
}
} // namespace tl::fea::beam18
