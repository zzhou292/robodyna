// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/t3/T3OnePointForce.h"
#include "lib_src/elements/t3/T3Startup.h"
#include "lib_utest/qualification/qbat/source_fixture/YarisQbatSourceFixture.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <gtest/gtest.h>

namespace t3_one_point_test {
namespace t3=tl::fea::t3;
namespace mat=tl::material;
constexpr double Dt=0x1p-15;
inline t3::Vec3 Add(t3::Vec3 a,t3::Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline t3::Vec3 Sub(t3::Vec3 a,t3::Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline t3::Vec3 Scale(t3::Vec3 a,double s) { return {a.x*s,a.y*s,a.z*s}; }
inline t3::Vec3 Cross(t3::Vec3 a,t3::Vec3 b) {
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
inline t3::Vec3 Transform(t3::Vec3 value,double angle) {
  const t3::Vec3 axis{1./3.,2./3.,2./3.};
  const double c=std::cos(angle),s=std::sin(angle);
  const double dot=value.x*axis.x+value.y*axis.y+value.z*axis.z;
  return Add(Add(Scale(value,c),Scale(Cross(axis,value),s)),Scale(axis,(1-c)*dot));
}
struct Fixture {
  t3::ReferenceData reference;
  t3::OnePointMaterial material;
  t3::OnePointFailure failure{2.5};
  explicit Fixture(bool transformed=false) {
    t3::ReferenceInput input;
    input.node_ids[0]=2300357;
    input.node_ids[1]=2300138;
    input.node_ids[2]=2300139;
    for (unsigned n=0;n<3;++n) {
      bool found=false;
      for (const auto& node:yaris_qbat_source_fixture::nodes) if (node.id==input.node_ids[n]) {
        input.position[n]={node.position_m[0],node.position_m[1],node.position_m[2]};
        if (transformed) input.position[n]=Transform(input.position[n],.73);
        found=true;
      }
      EXPECT_TRUE(found);
    }
    input.density=1000;
    input.young_modulus=250e6;
    input.poisson_ratio=.35;
    input.thickness=.0005;
    EXPECT_EQ(t3::InitializeReference(input,reference),t3::Status::kSuccess);
    const mat::TabulatedShellPlasticityRate rate{
        true,0,1,10000,mat::ShellPlasticityRatePolicy::FilteredZeroC};
    EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(250e6,.35,1000,{10e6,1e6},rate,material),
        mat::TabulatedShellPlasticityStatus::Ok);
  }
  t3::OnePointHistory Virgin() const {
    t3::OnePointHistory history;
    EXPECT_EQ(t3::InitializeOnePointLaw44History(reference,material,failure,{0,0},history),
        t3::Status::kSuccess);
    return history;
  }
};
// One independently prescribed smooth path: endpoint x and midpoint v/omega.
// Nonuniform director spin is deliberate; it is not inferred from nodal q.
inline t3::PrescribedInterval Path(const Fixture& fixture,unsigned step) {
  t3::PrescribedInterval in;
  in.base_time=step*Dt;
  in.dt=Dt;
  in.sample_index=step+1;
  const double te=(step+1)*Dt,tm=(step+.5)*Dt;
  constexpr double frequency=2*3.14159265358979323846/(128*Dt);
  const double ee=.18*std::sin(frequency*te),em=.18*std::sin(frequency*tm);
  const double de=.18*frequency*std::cos(frequency*tm);
  const double rotation_rate=1.2/(128*Dt);
  const t3::Vec3 omega{rotation_rate/3,2*rotation_rate/3,2*rotation_rate/3};
  t3::Vec3 center{};
  for (auto x:fixture.reference.input.position) center=Add(center,Scale(x,1./3.));
  for (unsigned n=0;n<3;++n) {
    const auto x=Sub(fixture.reference.input.position[n],center);
    const t3::Vec3 endpoint{(1+ee)*x.x+.31*ee*x.y,(1-.23*ee)*x.y,x.z+.13*ee*x.x};
    const t3::Vec3 midpoint{(1+em)*x.x+.31*em*x.y,(1-.23*em)*x.y,x.z+.13*em*x.x};
    const t3::Vec3 derivative{de*x.x+.31*de*x.y,-.23*de*x.y,.13*de*x.x};
    const auto world=Transform(midpoint,rotation_rate*tm);
    in.position[n]=Add(center,Transform(endpoint,rotation_rate*te));
    in.velocity[n]=Add(Transform(derivative,rotation_rate*tm),Cross(omega,world));
    in.angular_velocity[n]=Add(omega,Transform({60.*n,-45.*n,25.*n},rotation_rate*tm));
  }
  return in;
}
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> result;
  std::memcpy(result.data(),&value,sizeof(value));
  return result;
}
inline std::array<double,37> StateValues(const t3::OnePointHistoryValues& value) {
  std::array<double,37> result{};
  unsigned i=0;
  const auto& h=value.shell;
  for (double x:h.stress) result[i++]=x;
  for (double x:h.material_stress) result[i++]=x;
  for (double x:h.bending_stress) result[i++]=x;
  for (double x:h.strain_curvature) result[i++]=x;
  result[i++]=h.thickness;
  for (double x:h.internal_work) result[i++]=x;
  result[i++]=h.equivalent_strain_rate;
  result[i++]=h.active;
  for (double x:value.point.stress) result[i++]=x;
  result[i++]=value.point.plastic_strain;
  result[i++]=value.point.filtered_rate_per_s;
  result[i++]=value.failure.damage;
  result[i++]=value.failure.failure_time_s;
  result[i++]=value.failure.point_active?1:0;
  result[i++]=value.plastic_work_j;
  return result;
}
inline t3::OnePointHistory NearFailure(const Fixture& f) {
  auto values=f.Virgin().values();
  values.point.plastic_strain=2.49999;
  values.failure.damage=.9999999;
  const double stress=f.material.linear.initial_yield_pa+
      f.material.plastic_hardening_pa*values.point.plastic_strain;
  for (unsigned i=0;i<2;++i) {
    values.point.stress[i]=stress;
    values.shell.stress[i]=stress;
    values.shell.material_stress[i]=stress;
  }
  t3::OnePointHistory history;
  EXPECT_EQ(t3::PrepareOnePointLaw44History(f.reference,f.material,f.failure,values,{0,0},history),
      t3::Status::kSuccess);
  return history;
}
} // namespace t3_one_point_test
