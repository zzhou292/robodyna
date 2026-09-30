// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/qbat/QbatGeometry.h"
#include <array>
#include <cmath>

namespace qbat_test {
namespace qb = tl::fea::qbat;
inline qb::ReferenceInput Fixture(bool warped=true) {
  qb::ReferenceInput input;
  auto& q=input.quadrilateral;
  q.position[0]={0,0,0};
  q.position[1]={.04,0,warped?.001:0};
  q.position[2]={warped?.034:.04,.02,warped?-.0004:0};
  q.position[3]={warped?-.003:0,warped?.017:.02,warped?.0007:0};
  q.density=1100;
  q.young_modulus=250e6;
  q.poisson_ratio=.3;
  q.thickness=.00076;
  input.initial_a11_pa=q.young_modulus/(1-q.poisson_ratio*q.poisson_ratio);
  return input;
}
inline qb::CurrentInput Current(const qb::ReferenceInput& input) {
  qb::CurrentInput current;
  for (unsigned i=0;i<4;++i) current.position_m[i]=input.quadrilateral.position[i];
  return current;
}
inline qb::Vec3 Transform(qb::Vec3 x,double angle,double shift=0) {
  // Dense constant axis (1,2,2)/3; independent Rodrigues fixture transform.
  const double c=std::cos(angle),s=std::sin(angle),d=(x.x+2*x.y+2*x.z)/3;
  return {c*x.x+s*(2*x.z-2*x.y)/3+(1-c)*d/3+shift,
      c*x.y+s*(2*x.x-x.z)/3+(1-c)*2*d/3-2*shift,
      c*x.z+s*(x.y-2*x.x)/3+(1-c)*2*d/3+3*shift};
}
inline std::array<double,40> Values(const qb::Reference& reference) {
  std::array<double,40> result{};
  const auto& q=reference.quadrilateral();
  unsigned i=0;
  for(double x:q.frame.v) result[i++]=x;
  result[i++]=q.area;
  for(double x:q.derivative_x) result[i++]=x;
  for(double x:q.derivative_y) result[i++]=x;
  for(auto x:q.local_position) { result[i++]=x.x; result[i++]=x.y; result[i++]=x.z; }
  result[i++]=q.nodal_mass[0];
  result[i++]=q.physical_inertia[0];
  result[i++]=q.added_inertia[0];
  result[i++]=q.isotropic_inertia[0];
  const auto& k=reference.coefficients();
  result[i++]=k.characteristic_length_m;
  result[i++]=k.sound_speed_m_s;
  result[i++]=k.viscosity_timestep_factor;
  result[i++]=k.unscaled_element_dt_s;
  result[i++]=k.nodal_translation_stiffness_n_m;
  result[i++]=k.nodal_rotation_stiffness_nm;
  return result;
}
inline std::array<double,84> Values(const qb::Geometry& g) {
  std::array<double,84> result{};
  unsigned i=0;
  for(double x:g.frame.v) result[i++]=x;
  result[i++]=g.area_m2;
  result[i++]=g.reciprocal_area_per_m2;
  result[i++]=g.actual_warpage_m;
  for(auto x:g.centered_projected_position_m) { result[i++]=x.x; result[i++]=x.y; result[i++]=x.z; }
  for(double x:g.native_vcore) result[i++]=x;
  for(const auto& p:g.point) {
    result[i++]=p.jacobian_m2;
    result[i++]=p.hx_per_m;
    result[i++]=p.hy_per_m;
    for(double x:p.membrane_b_per_m) result[i++]=x;
  }
  for(double x:g.assumed_shear_per_m) result[i++]=x;
  return result;
}
} // namespace qbat_test
