// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
extern "C" void qbf_native_force(int,int,int,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,double*,double*);
namespace qbat_force_test {
void NativeState::Step(const Fixture& f,const qb::PrescribedInterval& in) {
  const auto& p=f.material;
  const int mfunc=p.hardening==mat::ShellPlasticityHardeningKind::LinearLaw44?0:1;
  const int npts=p.curve.count;
  std::vector<double> curve(2*(npts+1));
  for(int i=0;i<npts;++i) {
    curve[2*(i+1)]=p.curve.plastic_strain[i];
    curve[2*(i+1)+1]=p.curve.yield_stress_pa[i];
  }
  const double basic[]{p.density_kg_m3,p.young_pa,p.poisson_ratio};
  const double linear[]{p.linear.initial_yield_pa,p.linear.tangent_modulus_pa};
  const double controls[]{p.rate.cowper_symonds_c_per_s,p.rate.cowper_symonds_p,p.rate.cutoff_hz};
  std::array<double,12> x{},v{},omega{};
  for(unsigned i=0;i<4;++i) {
    x[3*i]=in.position_endpoint[i].x;x[3*i+1]=in.position_endpoint[i].y;x[3*i+2]=in.position_endpoint[i].z;
    v[3*i]=in.velocity_midpoint[i].x;v[3*i+1]=in.velocity_midpoint[i].y;v[3*i+2]=in.velocity_midpoint[i].z;
    omega[3*i]=in.omega_midpoint[i].x;omega[3*i+1]=in.omega_midpoint[i].y;omega[3*i+2]=in.omega_midpoint[i].z;
  }
  const auto& options=f.input.options;
  const double endpoint=in.base_time+in.dt;
  qbf_native_force(mfunc,npts,p.rate.enabled?1:0,curve.data(),basic,linear,controls,
      &options.membrane_viscosity,&options.numerical_viscosity,&f.failure.failure_strain,
      &in.dt,&endpoint,x.data(),v.data(),omega.data(),state.data(),output.data());
}
void CompareNative(const qb::ForceTrial& actual,const NativeState& native) {
  const auto state=StateValues(actual.proposed_history.data());
  for(unsigned i=0;i<state.size();++i) {
    SCOPED_TRACE("state "+std::to_string(i));
    // Stress and cache stress use Pa; the remaining history fields retain
    // their own SI scale. Active flags are checked exactly below.
    const unsigned field=i%24;
    const bool stress=i<96?(field<5 || (field>=11 && field<16)):(i<101);
    Close(state[i],native.state[i],stress?1e-6:2e-13,2e-10);
  }
  for(unsigned p=0;p<4;++p) {
    EXPECT_EQ(state[24*p+9],native.state[24*p+9]);
    EXPECT_EQ(state[24*p+10],native.state[24*p+10]);
  }
  EXPECT_EQ(state[115],native.state[115]);
  unsigned i=0;
  for(const auto& v:actual.internal_force_n) {
    Close(v.x,native.output[i++],2e-9);
    Close(v.y,native.output[i++],2e-9);
    Close(v.z,native.output[i++],2e-9);
  }
  for(const auto& v:actual.internal_couple_nm) {
    Close(v.x,native.output[i++]);Close(v.y,native.output[i++]);Close(v.z,native.output[i++]);
  }
  const auto geometry=qbat_test::Values(actual.kinematics.geometry);
  for(double v:geometry) Close(v,native.output[i++],2e-11);
  for(const auto& v:actual.kinematics.corrected_velocity) {
    Close(v.x,native.output[i++]);Close(v.y,native.output[i++]);Close(v.z,native.output[i++]);
  }
  for(double v:actual.kinematics.local_spin) Close(v,native.output[i++],2e-11);
  for(const auto& p:actual.kinematics.rate) for(double v:p) Close(v,native.output[i++],2e-10);
  for(const auto& p:actual.kinematics.strain_increment) for(double v:p) Close(v,native.output[i++],2e-13);
  for(unsigned p=0;p<4;++p) {
    const auto& a=actual.point[p].material;
    const unsigned n=189+p*13;
    for(unsigned c=0;c<5;++c) Close(a.history.stress[c],native.output[n+c],1e-6);
    Close(a.history.plastic_strain,native.output[n+5]);
    Close(a.plastic_increment,native.output[n+6]);
    Close(a.tangent_ratio,native.output[n+7]);
    Close(a.yield_before_pa,native.output[n+9],1e-6);
    Close(a.history.filtered_rate_per_s,native.output[n+12],2e-10);
    const double t[]{actual.point[p].thickness_before_m,actual.point[p].thickness_material_m,
      actual.point[p].thickness_after_m,actual.point[p].force_volume_m3};
    for(unsigned c=0;c<4;++c) Close(t[c],native.output[241+4*p+c],2e-15);
  }
  const auto& d=actual.diagnostics;
  const double coefficients[]{actual.kinematics.characteristic_length_m,
    actual.kinematics.nodal_factor[0],actual.kinematics.nodal_factor[1],
    d.numerical_viscosity,d.sound_speed_m_s,d.viscosity_timestep_factor,
    d.unscaled_element_dt_s,d.translation_stiffness_n_m,d.rotation_stiffness_nm};
  for(unsigned c=0;c<9;++c) Close(coefficients[c],native.output[257+c],2e-11);
}
} // namespace qbat_force_test
