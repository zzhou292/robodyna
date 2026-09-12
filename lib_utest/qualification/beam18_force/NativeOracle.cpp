// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "lib_utest/qualification/beam18_reference/NativeOracle.h"
#include <vector>
extern "C" void beam18_force_native(const double*,const double*,int,const double*,
    const double*,const int*,const double*,const double*,const double*,const double*,const double*,
    int,double*,int*,int*);
namespace beam18_force_test {
NativeResult Native(const b::Reference& reference,const b::Material& material,const NativeState& accepted,
    const b::PrescribedInterval& motion,bool initial) {
  NativeResult result;result.next=accepted;
  if(initial) {
    const auto native=beam18_test::Native(reference.input());
    if(native.status)throw std::runtime_error("native reference rejected");
    std::copy_n(native.values.begin(),27,result.next.reference.begin());
  }
  const bool working=reference.input().units==b::WorkingUnits::TonneMillimetreSecond;
  const double length=working?.001:1.,stress=working?1e6:1.;
  const double force=working?1.:1.,moment=working?.001:1.,stiffness=working?1000.:1.;
  double x[6],v[6],r[6];
  for(unsigned n=0;n<2;++n) {
    const auto p=initial ? reference.input().position[n] :
        tl::math::fixed3::Divide(motion.position_endpoint_m[n],length);
    const auto speed=tl::math::fixed3::Divide(motion.velocity_midpoint_m_s[n],length);
    const auto omega=motion.angular_velocity_midpoint_rad_s[n];
    x[3*n]=p.x;x[3*n+1]=p.y;x[3*n+2]=p.z;
    v[3*n]=speed.x;v[3*n+1]=speed.y;v[3*n+2]=speed.z;
    r[3*n]=omega.x;r[3*n+1]=omega.y;r[3*n+2]=omega.z;
  }
  const auto& m=material.material;
  const double raw[]{reference.input().young,m.poisson_ratio,reference.input().density,
      m.rate_c_per_s,m.rate_p,m.cutoff_hz==10000?0:m.cutoff_hz,270e6/stress};
  std::vector<double> curve(2*(material.curve.count+1));
  for(unsigned i=0;i<material.curve.count;++i) {
    curve[2*(i+1)]=material.curve.plastic_strain[i];
    curve[2*(i+1)+1]=material.curve.yield_stress_pa[i]/stress;
  }
  std::array<double,85> values{};
  beam18_force_native(result.next.reference.data(),raw,material.curve.count,curve.data(),accepted.history.data(),
      accepted.cursor.data(),x,v,r,&motion.base_time_s,&motion.dt_s,initial,values.data(),
      result.next.cursor.data(),&result.status);
  if(result.status)return result;
  std::copy_n(values.begin(),41,result.next.history.begin());
  result.si=values;
  for(unsigned p=0;p<4;++p)for(unsigned k=0;k<3;++k)result.si[7*p+k]*=stress;
  for(unsigned k=31;k<34;++k)result.si[k]*=force;
  for(unsigned k=34;k<37;++k)result.si[k]*=moment;
  for(unsigned k=38;k<41;++k)result.si[k]*=moment;
  for(unsigned k=41;k<47;++k)result.si[k]*=force;
  for(unsigned k=47;k<53;++k)result.si[k]*=moment;
  result.si[62]*=length;
  for(unsigned k=66;k<69;++k)result.si[k]/=length;
  result.si[69]*=stiffness;result.si[70]*=moment;
  for(unsigned k=72;k<75;++k)result.si[k]*=force;
  for(unsigned k=75;k<78;++k)result.si[k]*=moment;
  for(unsigned k=78;k<81;++k)result.si[k]*=stiffness;
  for(unsigned k=81;k<84;++k)result.si[k]*=moment;
  for(unsigned k=0;k<3;++k) {
    const double old_value=initial?0.:accepted.history[38+k];
    result.work_increment_j[k]=(values[38+k]-old_value)*moment;
    result.work_difference_scale_j[k]=(std::abs(values[38+k])+std::abs(old_value))*moment;
  }
  return result;
}
std::array<double,85> Values(const b::ForceTrial& trial) {
  std::array<double,85> v{};const auto& h=trial.proposed_history.values();unsigned next=0;
  auto vector=[&](b::Vec3 a){v[next++]=a.x;v[next++]=a.y;v[next++]=a.z;};
  for(unsigned p=0;p<4;++p) {
    for(double x:h.point[p].stress_pa)v[next++]=x;
    for(double x:h.total_strain[p])v[next++]=x;
    v[next++]=h.point[p].plastic_strain;
  }
  vector(h.section_seed);vector(h.section_force_n);vector(h.section_moment_nm);
  v[next++]=h.filtered_neutral_rate_per_s;
  v[next++]=h.internal_energy_j[0];v[next++]=h.internal_energy_j[1];v[next++]=h.plastic_work_j;
  for(auto x:trial.rhs_force_n)vector(x);
  for(auto x:trial.rhs_couple_nm)vector(x);
  for(auto x:trial.geometry.axis)vector(x);
  v[next++]=trial.geometry.length_m;
  const auto& r=trial.rate;
  for(double x:{r.axial,r.shear_y,r.shear_z,r.curvature_x,r.curvature_y,r.curvature_z})v[next++]=x;
  const auto& d=trial.diagnostics;
  v[next++]=d.translation_stiffness_n_m;v[next++]=d.rotation_stiffness_nm;v[next++]=d.minimum_unscaled_dt_s;
  vector(d.damped_section_force_n);vector(d.damped_section_moment_nm);
  for(double x:{d.translation_stiffness_n_m,d.translation_stiffness_n_m,0.,d.rotation_stiffness_nm,d.rotation_stiffness_nm,0.,1.})v[next++]=x;
  EXPECT_EQ(next,v.size());return v;
}
void Compare(const b::ForceTrial& actual,const NativeResult& expected) {
  ASSERT_EQ(expected.status,0);
  const auto values=Values(actual);
  for(unsigned i=0;i<values.size();++i) {
    SCOPED_TRACE(i);
    EXPECT_NEAR(values[i],expected.si[i],3e-10*std::max({std::abs(values[i]),std::abs(expected.si[i]),1e-20}));
  }
  for(unsigned p=0;p<4;++p)EXPECT_EQ(actual.proposed_history.values().point[p].curve_cursor,unsigned(expected.next.cursor[p]));
  const auto& d=actual.diagnostics;
  const double increment[]{d.internal_work_increment_j[0],d.internal_work_increment_j[1],d.plastic_work_increment_j};
  for(unsigned k=0;k<3;++k) {
    SCOPED_TRACE(k);
    // Difference of separately checked cumulative histories can cancel.
    EXPECT_NEAR(increment[k],expected.work_increment_j[k],
        3e-10*std::max(expected.work_difference_scale_j[k],1e-20));
  }
}
} // namespace beam18_force_test
