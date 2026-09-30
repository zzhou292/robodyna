#pragma once
// Test-only independent continuum material truths and the passed R2 analytic
// rate map. No native force/material calls or production constitutive helper.
#include "T3ForceReference.h"
#include "T3KinematicsTestOracle.h"
#include <limits>

namespace tl::qualification::t3::force_test {
namespace kt=kinematic_test;
inline void Near(double actual,long double expected,long double absolute=2e-11L) {
  EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),
      absolute+2e-10L*std::max(std::abs(static_cast<long double>(actual)),std::abs(expected)));
}
inline History MakeHistory(const Reference& r,HistoryValues h={},HistoryStamp stamp={}) {
  if(h.thickness==0) h.thickness=r.data().input.thickness;
  History out;
  if(PreparePrescribedHistory(r,h,stamp,out)!=Status::kSuccess) throw std::runtime_error("T3 test history rejected");
  return out;
}
inline HistoryValues Seed(double t) {
  HistoryValues h; h.thickness=t; h.active=1; h.equivalent_strain_rate=.7;
  h.stress={137,-211,89,17,-23}; h.material_stress={123,-207,81,17,-23};
  h.bending_stress={-31,27,13}; h.internal_work={.375,-.125};
  for(unsigned k=0;k<8;++k) h.strain_curvature[k]=(.25+k)*1e-6;
  return h;
}
struct Oracle { std::array<long double,26> values{}; };
inline Oracle Independent(const ReferenceInput& r,const HistoryValues& base,const PrescribedInterval& in) {
  Oracle out; const auto geometry=kt::Independent(in); const auto area=geometry.area;
  const long double E=r.young_modulus,nu=r.poisson_ratio,t=r.thickness;
  const long double g=E/(2*(1+nu)),a11=E/(1-nu*nu),a12=nu*a11,gs=g*5/6;
  std::array<long double,8> d{};
  for(unsigned k=0;k<8;++k) { d[k]=geometry.normalized[k]*in.dt; out.values[13+k]=base.strain_curvature[k]+d[k]; }
  std::array<long double,5> material{base.material_stress[0]+a11*d[0]+a12*d[1],
      base.material_stress[1]+a12*d[0]+a11*d[1],base.material_stress[2]+g*d[2],
      base.material_stress[3]+gs*d[3],base.material_stress[4]+gs*d[4]};
  const std::array<long double,3> moment{base.bending_stress[0]+t/12*(a11*d[5]+a12*d[6]),
      base.bending_stress[1]+t/12*(a12*d[5]+a11*d[6]),base.bending_stress[2]+t/12*g*d[7]};
  const long double inverse_dt=in.dt/std::max(static_cast<long double>(in.dt)*in.dt,1e-20L);
  const long double visc=1.414L*.015L*r.density*std::sqrt(E/std::max(static_cast<long double>(r.density),1e-20L))
      *std::sqrt(area)*inverse_dt;
  auto total=material;
  total[0]+=visc*(d[0]+.5L*d[1]); total[1]+=visc*(d[1]+.5L*d[0]); total[2]+=visc*d[2]/3;
  long double membrane=0,bending=0;
  for(unsigned k=0;k<5;++k) {
    out.values[k]=total[k]; out.values[k+5]=material[k]; membrane+=(base.stress[k]+total[k])*d[k];
  }
  for(unsigned k=0;k<3;++k) { out.values[10+k]=moment[k]; bending+=(base.bending_stress[k]+moment[k])*d[k+5]; }
  out.values[21]=base.thickness*(1-nu*(d[0]+d[1])/(1-nu));
  out.values[22]=base.internal_work[0]+membrane*.5L*t*area;
  out.values[23]=base.internal_work[1]+bending*.5L*t*t*area;
  out.values[24]=std::sqrt((d[5]*d[5]+d[6]*d[6]+d[5]*d[6]+d[7]*d[7]/4)*base.thickness*base.thickness/9+
      4.L/3*(d[0]*d[0]+d[1]*d[1]+d[0]*d[1]+d[2]*d[2]/4))*inverse_dt;
  out.values[25]=1;
  return out;
}
inline void Check(const Reference& r,const HistoryValues& base,const PrescribedInterval& in,const ForceTrial& out) {
  kt::Check(out.kinematics,in);
  std::array<double,26> actual{}; detail::PackHistory(out.proposed_history.data(),actual);
  const auto expected=Independent(r.data().input,base,in);
  for(unsigned k=0;k<26;++k) { SCOPED_TRACE(k); Near(actual[k],expected.values[k],k==22||k==23?2e-22L:2e-11L); }
  for(unsigned k=0;k<2;++k) Near(out.diagnostics.internal_work_increment[k],
      expected.values[22+k]-base.internal_work[k],2e-22L+64*std::numeric_limits<double>::epsilon()*
      (std::abs(base.internal_work[k])+std::abs(out.proposed_history.data().internal_work[k])));
  EXPECT_EQ(out.diagnostics.effective_thickness,r.data().input.thickness);
  EXPECT_EQ(out.proposed_history.stamp().time,in.base_time+in.dt);
  EXPECT_EQ(out.proposed_history.stamp().sample_index,in.sample_index);
}
inline void CheckFixedResultantPower(const ReferenceInput& r,const ForceTrial& out) {
  // Independent virtual power of the linearized rate map, holding current
  // resultants fixed. dt0 belongs only to this analytic test oracle.
  const auto& h=out.proposed_history.data();
  const long double t=r.thickness;
  for(unsigned column=0;column<18;++column) {
    auto virtual_state=kt::Interval(r); virtual_state.dt=0;
    const unsigned n=column/6,axis=column%3;
    auto& v=column%6<3?virtual_state.velocity[n]:virtual_state.angular_velocity[n];
    if(axis==0)v.x=1; else if(axis==1)v.y=1; else v.z=1;
    const auto k=kt::Independent(virtual_state);
    long double power=0;
    for(unsigned j=0;j<5;++j) power+=t*h.stress[j]*k.raw[j];
    for(unsigned j=0;j<3;++j) power+=t*t*h.bending_stress[j]*k.raw[j+5];
    const auto& f=column%6<3?out.internal_force[n]:out.internal_couple[n];
    Near(axis==0?f.x:axis==1?f.y:f.z,power);
  }
}
inline void Mode(PrescribedInterval& in,unsigned mode,double rate) {
  const auto geometry=kt::Independent(in);
  for(unsigned n=0;n<3;++n) {
    const double x=double(geometry.local[n][0]),y=double(geometry.local[n][1]);
    kt::Wide v{},w{};
    if(mode==0)v[0]=rate*x; if(mode==1)v[1]=rate*y; if(mode==2)v[0]=rate*y;
    if(mode==3)v[2]=rate*y; if(mode==4)v[2]=rate*x;
    if(mode==5)w[1]=rate*x; if(mode==6)w[0]=-rate*y; if(mode==7)w[1]=rate*y;
    kt::Wide world_v{},world_w{};
    for(unsigned axis=0;axis<3;++axis) {
      world_v=kt::Add(world_v,kt::Scale(geometry.basis[axis],v[axis]));
      world_w=kt::Add(world_w,kt::Scale(geometry.basis[axis],w[axis]));
    }
    in.velocity[n]=kt::Narrow(world_v); in.angular_velocity[n]=kt::Narrow(world_w);
  }
}
} // namespace tl::qualification::t3::force_test
