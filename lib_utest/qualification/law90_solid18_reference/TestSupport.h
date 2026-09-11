// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/total_strain/Reference.h"
#include "lib_src/elements/solid18/total_strain/TotalKinematics.h"
#include "lib_utest/qualification/solid18_reference/TestSupport.h"
#include <limits>
namespace law90_reference_test {
namespace s = tl::fea::solid18;
namespace t = s::total_strain;
using solid18_test::Bytes;
inline s::ReferenceInput Cube() {
  auto input = solid18_test::Cube();
  input.profile = t::Law90Profile();
  input.density_kg_m3 = 772;
  return input;
}
inline s::ReferenceInput Distorted() {
  auto input = solid18_test::Distorted();
  input.profile = t::Law90Profile();
  input.density_kg_m3 = 772;
  return input;
}
inline t::KinematicsInput Current(const s::ReferenceInput& input) {
  t::KinematicsInput result;
  for (unsigned n = 0; n < 8; ++n) result.position_m[n] = input.position_m[n];
  return result;
}
inline t::KinematicsInput Path(const s::ReferenceInput& input, double phase) {
  auto result = Current(input);
  result.dt_s = 1e-4;
  const double c = std::cos(phase*.21), sn = std::sin(phase*.21);
  const auto origin=input.position_m[0];
  for (unsigned n = 0; n < 8; ++n) {
    const auto position=input.position_m[n];
    const s::Vec3 x{position.x-origin.x,position.y-origin.y,position.z-origin.z};
    const double a = (1-.07*phase)*x.x+.04*phase*x.y*x.z;
    const double b = (1+.03*phase)*x.y+.02*phase*x.z*x.x;
    result.position_m[n] = {c*a-sn*b+origin.x+.31, sn*a+c*b+origin.y-.27,
                           x.z*(1-.02*phase)+.03*phase*x.x*x.y+origin.z+.13};
    result.velocity_m_s[n] = {-.17*x.x+.21*x.y*x.z, .11*x.y-.08*x.z*x.x,
                              .13*x.z+.16*x.x*x.y};
  }
  return result;
}
inline constexpr unsigned ReferenceCount = 755;
inline std::array<double,ReferenceCount> ReferenceValues(const t::Reference& r) {
  std::array<double,ReferenceCount> result{};
  unsigned i=0;
  const auto put=[&](const s::Vec3& v){result[i++]=v.x; result[i++]=v.y; result[i++]=v.z;};
  const auto& g=r.geometry();
  for (double x:g.frame.v) result[i++]=x;
  for (const auto& x:g.native_position_m) put(x);
  for (double x:g.center_scaled_jacobian_m.v) result[i++]=x;
  for (const auto& x:g.higher_mode_m) put(x);
  for (const auto& p:g.point) {
    for (double x:p.scaled_jacobian_m.v) result[i++]=x;
    result[i++]=p.initial_volume_m3;
  }
  result[i++]=g.center_volume_m3;
  result[i++]=g.integrated_volume_m3;
  result[i++]=g.inverse_center_face_scale_per_m2;
  result[i++]=g.characteristic_length_m;
  for (double x:r.mass().source_nodal_mass_kg) result[i++]=x;
  result[i++]=r.mass().element_mass_kg;
  result[i++]=r.mass().initial_global_density_kg_m3;
  for (double x:r.coefficients().center_jacobian_inverse_per_m) result[i++]=x;
  result[i++]=r.coefficients().center_determinant_m3;
  for (const auto& p:r.coefficients().point_pij_per_m)
    for (double x:p) result[i++]=x;
  for (const auto& x:r.coefficients().source_relative_position_m) put(x);
  assert(i==result.size());
  return result;
}
inline constexpr unsigned CurrentCount = 991; // 71 geometry +8*82 point derivatives +24 displacement +8*30.
inline std::array<double,CurrentCount> CurrentValues(const t::Kinematics& k) {
  std::array<double,CurrentCount> result{};
  unsigned i=0;
  const auto put=[&](const s::Vec3& v){result[i++]=v.x; result[i++]=v.y; result[i++]=v.z;};
  const auto& g=k.geometry;
  for(double x:g.frame.v) result[i++]=x;
  for(const auto& x:g.local_position_m) put(x);
  for(const auto& x:g.local_velocity_m_s) put(x);
  for(const auto& row:g.center_gradient_per_m) for(double x:row) result[i++]=x;
  result[i++]=g.center_volume_m3;
  result[i++]=g.inverse_center_face_scale_per_m2;
  for(const auto& p:g.point) {
    result[i++]=p.current_volume_m3;
    for(double x:p.inverse_scaled_jacobian_per_m.v) result[i++]=x;
    for(const auto& row:p.regular_per_m) for(double x:row) result[i++]=x;
    for(const auto& row:p.shear_per_m) for(double x:row) result[i++]=x;
  }
  for(const auto& x:k.reference_displacement_m) put(x);
  for(const auto& p:k.point) {
    for(double x:p.world_displacement_gradient) result[i++]=x;
    for(double x:p.material_displacement_gradient) result[i++]=x;
    for(double x:p.selected_left_cauchy_green_minus_identity) result[i++]=x;
    for(double x:p.engineering_rate_per_s) result[i++]=x;
  }
  assert(i==result.size());
  return result;
}
}
