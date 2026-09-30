// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type13/Type13Math.h"
#include "lib_utest/qualification/type13/Fixture.h"
#include <array>
namespace type13_recurrence_test {
namespace t=tl::fea::type13;
namespace f=tl::math::fixed3;
using type13_test::Fixture;
struct Case {
  t::Property property{};t::Reference reference{};t::ReferenceInput original{};
  t::NativeEndpointKinematics nodes[2]{};
  Case(bool dense=false,double failure=2e20) {
    Fixture fixture;auto input=fixture.Input();
    for(auto& c:input.channels){c.failure_negative=-failure;c.failure_positive=failure;}
    if(t::InitializeProperty(input,property)!=t::Status::Success)return;
    original=dense?type13_test::DenseReference():t::ReferenceInput{};
    if(!dense){original.position[0]={0,0,0};original.position[1]={2,0,0};original.position[2]={0,1,0};}
    t::Startup startup;
    if(t::InitializeElement(property,original,startup)!=t::Status::Success)return;
    reference=startup.reference;
    for(unsigned k=0;k<2;++k)nodes[k].position=original.position[k];
  }
};
inline std::array<double,90> Values(const t::Evaluation& e) {
  std::array<double,90> a{};std::size_t n=0;
  const auto vec=[&](t::Vec3 v){a[n++]=v.x;a[n++]=v.y;a[n++]=v.z;};
  vec(e.native_history.transverse_axis);
  for(const auto& c:e.native_history.channels){a[n++]=c.deformation;a[n++]=c.accumulated_plastic_deformation;
    a[n++]=c.elastic_plastic_force;a[n++]=c.force;a[n++]=c.signed_work;a[n++]=c.curve_position;}
  a[n++]=e.native_history.failure_criterion;
  for(double v:e.native_frame.axes.v)a[n++]=v;
  for(double v:e.native_frame.midpoint_axes.v)a[n++]=v;
  a[n++]=e.native_frame.length;a[n++]=e.native_frame.midpoint_length;
  for(const auto& w:e.endpoints){vec(w.force_N);vec(w.couple_Nm);}
  vec(e.local_force_N);vec(e.local_couple_Nm);
  for(double v:e.signed_work_J)a[n++]=v;
  a[n++]=e.total_signed_work_J;a[n++]=e.stability.critical_dt_s;
  a[n++]=e.stability.translation_stiffness_N_per_m;a[n++]=e.stability.rotation_stiffness_Nm_per_rad;
  a[n++]=e.native_history.active?1:0;a[n++]=e.newly_failed?1:0;
  return a;
}
// Prescribed endpoint-position increment and true interval velocity; the spin
// couple has zero mean so pure rotations do not fabricate frame twist.
inline void Mode(Case& c,unsigned channel,double amplitude,double previous,double dt) {
  for(unsigned k=0;k<2;++k)c.nodes[k]={c.original.position[k],{},{}};
  const auto axis=f::Column(c.reference.axes,channel%3);
  if(channel<3) {
    c.nodes[1].position=f::Add(c.nodes[1].position,f::Scale(axis,amplitude));
    c.nodes[1].velocity=f::Scale(axis,(amplitude-previous)/dt);
  } else {
    const auto w=f::Scale(axis,(amplitude-previous)/dt*.5);
    c.nodes[0].angular_velocity=f::Scale(w,-1);c.nodes[1].angular_velocity=w;
  }
}
} // namespace type13_recurrence_test
