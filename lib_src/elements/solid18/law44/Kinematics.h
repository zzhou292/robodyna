// SPDX-License-Identifier: AGPL-3.0-or-later
// DEGENES8, S8EDEFOC3 and selected ICP1/I_SH0 S8EDEFO3.
#pragma once
#include "ForceTypes.h"
#include "lib_src/elements/solid18/Solid18CurrentGeometry.h"
#include "lib_src/elements/solid18/Solid18RateValues.h"

namespace tl::fea::solid18::law44::detail {
TL_SOLID18_HD inline unsigned NativeDegeneracy(const Reference& reference) noexcept {
  // Exact DEGENES8 membership procedure; classify native source incidence,
  // independent of coordinates and of the admitted topology enum.
  std::uint64_t nodes[8];
  for (unsigned n = 0; n < 8; ++n) nodes[n] = reference.input().source_node_id[reference.source_slot(n)];
  unsigned count = 0;
  for (unsigned n = 0; n < 8; ++n) {
    const auto id = nodes[n];
    nodes[n] = 0;
    bool present = false;
    for (auto other : nodes) if (other == id) present = true;
    if (present) ++count;
    nodes[n] = id;
  }
  return count/2;
}
TL_SOLID18_HD inline double CenterDivergence(const CurrentGeometry& geometry) noexcept {
  const auto& p = geometry.center_gradient_per_m;
  const auto& v = geometry.local_velocity_m_s;
  return p[0][0]*(v[0].x-v[6].x)+p[0][1]*(v[1].x-v[7].x)+
      p[0][2]*(v[2].x-v[4].x)+p[0][3]*(v[3].x-v[5].x)+
      p[1][0]*(v[0].y-v[6].y)+p[1][1]*(v[1].y-v[7].y)+
      p[1][2]*(v[2].y-v[4].y)+p[1][3]*(v[3].y-v[5].y)+
      p[2][0]*(v[0].z-v[6].z)+p[2][1]*(v[1].z-v[7].z)+
      p[2][2]*(v[2].z-v[4].z)+p[2][3]*(v[3].z-v[5].z);
}
TL_SOLID18_HD inline Status GeometryValues(const Reference& reference,
    const PrescribedInterval& interval, CurrentGeometry& output, StartupGeometry& scratch) noexcept {
  Vec3 position[8], velocity[8];
  for (unsigned n = 0; n < 8; ++n) {
    const auto slot = reference.source_slot(n);
    position[n] = interval.position_endpoint_m[slot];
    velocity[n] = interval.velocity_midpoint_m_s[slot];
  }
  return solid18::detail::NativeCurrentGeometryValues(position,velocity,output,scratch);
}
TL_SOLID18_HD inline Status PointKinematics(const PointDerivatives& geometry,
    const Vec3 (&velocity)[8], double center_divergence, unsigned degeneracy,
    double dt, PointHistory& proposed, PointObservation& observation) noexcept {
  using solid18::detail::RateSum;
  const auto& p = geometry.regular_per_m;
  const double xy = RateSum(p[1],velocity,0);
  const double xz = RateSum(p[2],velocity,0);
  const double yx = RateSum(p[0],velocity,1);
  const double yz = RateSum(p[2],velocity,1);
  const double zx = RateSum(p[0],velocity,2);
  const double zy = RateSum(p[1],velocity,2);
  const double xx = RateSum(p[0],velocity,0);
  const double yy = RateSum(p[1],velocity,1);
  const double zz = RateSum(p[2],velocity,2);
  const double correction = (center_divergence-(xx+yy+zz))*dt;
  double dv = correction*1.0;
  if (degeneracy > 10) dv = 0;
  observation.selective_volume_increment = dv;
  if (dv > 1.0-1e-20) dv = 0;
  const double factor = 1.0-dv;
  observation.storage_volume_factor = factor;
  proposed.storage_volume_m3 = proposed.storage_volume_m3*factor;
  proposed.energy_density_j_m3 = proposed.energy_density_j_m3/factor;
  solid18::detail::QuadraticEngineeringRate(xx,yy,zz,xy,xz,yx,yz,zx,zy,dt,
                                           observation.engineering_rate_per_s);
  if (!solid18::detail::Positive(proposed.storage_volume_m3) ||
      !tl::math::Finite(proposed.energy_density_j_m3) || !tl::math::Finite(correction))
    return Status::NonfiniteResult;
  for (double rate : observation.engineering_rate_per_s) {
    if (!tl::math::Finite(rate)) return Status::NonfiniteResult;
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::law44::detail
