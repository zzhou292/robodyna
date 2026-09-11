// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/qbat/QbatForce.h"
#include "../qbat/Fixture.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <algorithm>
#include <limits>

namespace qbat_force_test {
namespace qb=tl::fea::qbat;
namespace mat=tl::material;
constexpr double Dt=0x1p-15;
struct Fixture {
  qb::ReferenceInput input;
  qb::Reference reference;
  qb::Material material;
  qb::Failure failure{2.5};
  Fixture(bool warped=true,double dm=0) {
    input=qbat_test::Fixture(warped);
    input.quadrilateral.density=1000;
    input.quadrilateral.poisson_ratio=.35;
    input.quadrilateral.thickness=.0005;
    input.initial_a11_pa=250e6/(1-.35*.35);
    input.options.membrane_viscosity=dm;
    EXPECT_EQ(qb::InitializeReference(input,reference),qb::Status::kSuccess);
    const mat::TabulatedShellPlasticityRate rate{
      true,0,1,10000,mat::ShellPlasticityRatePolicy::FilteredZeroC};
    EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(250e6,.35,1000,{10e6,1e6},rate,material),
        mat::TabulatedShellPlasticityStatus::Ok);
  }
  qb::History Virgin() const {
    qb::History h;
    EXPECT_EQ(qb::InitializeHistory(reference,material,failure,{0,0},h),qb::Status::kSuccess);
    return h;
  }
};
// Prescribed endpoint geometry and midpoint analytical velocity/spin share
// the same continuous path. No native/production update supplies the inputs.
inline qb::PrescribedInterval Path(const Fixture& f,unsigned step) {
  qb::PrescribedInterval p;
  p.base_time=step*Dt;
  p.dt=Dt;
  p.sample_index=step+1;
  if (step>=128 && step<144) {
    const auto end=Path(f,127);
    for(unsigned n=0;n<4;++n) p.position_endpoint[n]=end.position_endpoint[n];
    return p;
  }
  const unsigned phase_step=step<144?step:step-16;
  constexpr double frequency=2*3.14159265358979323846/(128*Dt);
  const double te=(phase_step+1)*Dt,tm=(phase_step+.5)*Dt;
  const double ee=.18*std::sin(frequency*te),em=.18*std::sin(frequency*tm);
  const double de=.18*frequency*std::cos(frequency*tm);
  const double ae=1.35*te/(128*Dt),am=1.35*tm/(128*Dt),da=1.35/(128*Dt);
  const qb::Vec3 omega{da/3,2*da/3,2*da/3};
  for (unsigned n=0;n<4;++n) {
    const auto x=f.input.quadrilateral.position[n];
    const double sign=(n==0 || n==2)?1.:-1.;
    const qb::Vec3 endpoint{(1+ee)*x.x+.21*ee*x.y,(1-.24*ee)*x.y,
      x.z+sign*.002*ee};
    const qb::Vec3 midpoint{(1+em)*x.x+.21*em*x.y,(1-.24*em)*x.y,
      x.z+sign*.002*em};
    const qb::Vec3 rate{de*x.x+.21*de*x.y,-.24*de*x.y,sign*.002*de};
    const auto world=qbat_test::Transform(midpoint,am);
    const auto velocity=qbat_test::Transform(rate,am);
    p.position_endpoint[n]=qbat_test::Transform(endpoint,ae);
    p.velocity_midpoint[n]={velocity.x+omega.y*world.z-omega.z*world.y,
      velocity.y+omega.z*world.x-omega.x*world.z,
      velocity.z+omega.x*world.y-omega.y*world.x};
    p.omega_midpoint[n]=omega;
  }
  return p;
}
inline std::array<double,116> StateValues(const qb::HistoryValues& h) {
  std::array<double,116> out{};
  unsigned i=0;
  for (const auto& p:h.point) {
    for(double x:p.material.stress) out[i++]=x;
    out[i++]=p.material.plastic_strain;
    out[i++]=p.material.filtered_rate_per_s;
    out[i++]=p.failure.damage;
    out[i++]=p.failure.failure_time_s;
    out[i++]=p.failure.point_active?1:0;
    out[i++]=p.surface_active?1:0;
    for(double x:p.force_stress_pa) out[i++]=x;
    for(double x:p.strain) out[i++]=x;
  }
  for(double x:h.force_stress_pa) out[i++]=x;
  for(double x:h.strain) out[i++]=x;
  out[i++]=h.thickness_m;
  for(double x:h.internal_work_j) out[i++]=x;
  out[i++]=h.plastic_work_j;
  out[i++]=h.numerical_viscous_work_j;
  out[i++]=h.reported_rate_per_s;
  out[i++]=h.element_active?1:0;
  return out;
}
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& v) {
  std::array<unsigned char,sizeof(T)> out{};
  std::memcpy(out.data(),&v,sizeof v);
  return out;
}
inline qb::History NearRemoval(const Fixture& f,unsigned last) {
  auto values=f.Virgin().data();
  for(unsigned n=0;n<4;++n) {
    auto& p=values.point[n];
    if(n==last) {
      p.failure.damage=1-1e-8;
      p.material.stress[0]=10e6;
    }
    else {
      p.failure.damage=1;
      p.failure.point_active=false;
      p.surface_active=false;
    }
  }
  qb::History result;
  EXPECT_EQ(qb::PreparePrescribedHistory(f.reference,f.material,f.failure,values,{0,0},result),
      qb::Status::kSuccess);
  return result;
}
inline void Close(double a,double b,double floor=1e-13,double relative=2e-10) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  EXPECT_NEAR(a,b,floor+relative*std::max(std::abs(a),std::abs(b)));
}
} // namespace qbat_force_test
