// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/Solid18Force.h"
#include "lib_utest/qualification/solid18_reference/TestSupport.h"
#include "lib_utest/qualification/solid_law36_point/source_fixture/OriginalMaterial.h"
#include <gtest/gtest.h>
#include <limits>

namespace solid18_force_test {
namespace s = tl::fea::solid18;
inline s::Material Material() {
  s::Material result;
  EXPECT_EQ(tl::material::law36::Prepare(law36_test::E,law36_test::Nu,law36_test::Rho,
      law36_test::Curve,result),tl::material::law36::Status::Ok);
  return result;
}
inline s::Reference Reference(bool distorted = true) {
  auto input = distorted ? solid18_test::Distorted() : solid18_test::Cube();
  for (auto& position : input.position_m) {
    position.x *= .01;
    position.y *= .008;
    position.z *= .006;
  }
  s::Reference result;
  EXPECT_EQ(s::InitializeReference(input,result),s::Status::Success);
  return result;
}
inline s::PrescribedInterval Path(const s::Reference& reference, unsigned step) {
  constexpr double dt = 1.0/1048576;
  const double end = (step+1)*dt;
  const double mid = (step+.5)*dt;
  const auto amplitude = [](double time) { return .05*std::sin(2*3.141592653589793*time/(320*dt)); };
  const auto rate = [](double time) {
    return .05*(2*3.141592653589793/(320*dt))*std::cos(2*3.141592653589793*time/(320*dt));
  };
  s::PrescribedInterval result;
  result.base_time_s = step*dt;
  result.dt_s = dt;
  result.sample_index = step+1;
  for (unsigned n = 0; n < 8; ++n) {
    const auto& x = reference.input().position_m[n];
    const s::Vec3 shape{x.x+.4*x.y+.2*x.x*x.z/.006,
                        -.3*x.y+.15*x.z,-.2*x.z+.13*x.x};
    const double a = amplitude(end);
    const double v = rate(mid);
    result.position_endpoint_m[n] = {x.x+a*shape.x,x.y+a*shape.y,x.z+a*shape.z};
    result.velocity_midpoint_m_s[n] = {v*shape.x,v*shape.y,v*shape.z};
  }
  return result;
}
inline bool SameHistory(const s::History& a, const s::History& b) {
  if (!a.prepared() || !b.prepared() || a.stamp().time_s != b.stamp().time_s ||
      a.stamp().sample_index != b.stamp().sample_index) return false;
  for (unsigned ip = 0; ip < 8; ++ip) {
    const auto& x = a.data().point[ip];
    const auto& y = b.data().point[ip];
    for (unsigned k = 0; k < 6; ++k) {
      if (!s::detail::SameScalar(x.material.point.stress_pa[k],y.material.point.stress_pa[k]) ||
          !s::detail::SameScalar(x.material.point.engineering_strain[k],
                                y.material.point.engineering_strain[k])) return false;
    }
    const double xv[] = {x.material.point.plastic_strain,x.material.point.deviatoric_rate_per_s,
        x.material.internal_energy_density_j_m3,x.material.plastic_work_j,x.density_kg_m3,
        x.storage_volume_m3,x.initial_volume_m3,x.bulk_pressure_pa};
    const double yv[] = {y.material.point.plastic_strain,y.material.point.deviatoric_rate_per_s,
        y.material.internal_energy_density_j_m3,y.material.plastic_work_j,y.density_kg_m3,
        y.storage_volume_m3,y.initial_volume_m3,y.bulk_pressure_pa};
    for (unsigned k = 0; k < 8; ++k) {
      if (!s::detail::SameScalar(xv[k],yv[k])) return false;
    }
  }
  const auto& x = a.data().global;
  const auto& y = b.data().global;
  for (unsigned k = 0; k < 6; ++k) {
    if (!s::detail::SameScalar(x.stress_pa[k],y.stress_pa[k])) return false;
  }
  const double xv[] = {x.plastic_strain,x.density_kg_m3,x.internal_energy_density_j_m3,
                       x.plastic_work_j,x.bulk_pressure_pa};
  const double yv[] = {y.plastic_strain,y.density_kg_m3,y.internal_energy_density_j_m3,
                       y.plastic_work_j,y.bulk_pressure_pa};
  for (unsigned k = 0; k < 5; ++k) {
    if (!s::detail::SameScalar(xv[k],yv[k])) return false;
  }
  for (unsigned n = 0; n < 7; ++n) {
    if (!s::detail::SameVector(a.data().saved_local_position_m[n],
                             b.data().saved_local_position_m[n])) return false;
  }
  return true;
}
}  // namespace solid18_force_test
