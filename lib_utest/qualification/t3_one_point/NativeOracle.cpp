// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
extern "C" void t3_one_point_native(int,int,int,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,
    const double*,double*,double*,int*);
namespace t3_one_point_test {
int NativeState::Step(const Fixture& f,const t3::PrescribedInterval& in) {
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
  std::array<double,9> x{},v{},omega{};
  for(unsigned i=0;i<3;++i) {
    x[3*i]=in.position[i].x;
    x[3*i+1]=in.position[i].y;
    x[3*i+2]=in.position[i].z;
    v[3*i]=in.velocity[i].x;
    v[3*i+1]=in.velocity[i].y;
    v[3*i+2]=in.velocity[i].z;
    omega[3*i]=in.angular_velocity[i].x;
    omega[3*i+1]=in.angular_velocity[i].y;
    omega[3*i+2]=in.angular_velocity[i].z;
  }
  const double endpoint=in.base_time+in.dt;
  auto next=state;
  std::array<double,126> result{};
  int status=-1;
  t3_one_point_native(mfunc,npts,p.rate.enabled?1:0,curve.data(),basic,linear,controls,
      &f.failure.failure_strain,&in.dt,&endpoint,x.data(),v.data(),omega.data(),
      next.data(),result.data(),&status);
  if(status==0) {
    state=next;
    output=result;
  }
  return status;
}
void CompareNative(const t3::OnePointForceTrial& a,const NativeState& b) {
  const auto state=StateValues(a.proposed_history.values());
  for(unsigned i=0;i<37;++i) {
    SCOPED_TRACE("history "+std::to_string(i));
    const bool stress=i<13 || (i>=26 && i<31);
    Close(state[i],b.state[i],stress?1e-6:2e-13);
  }
  EXPECT_EQ(state[25],b.state[25]);
  EXPECT_EQ(state[34],b.state[34]);
  EXPECT_EQ(state[35],b.state[35]);
  std::vector<double> geometry;
  const auto& k=a.kinematics;
  for(double x:k.frame.v) geometry.push_back(x);
  geometry.insert(geometry.end(),{k.area,k.characteristic_length,k.area_scale,
      k.local_position[1].x,k.local_position[1].y,k.local_position[2].x,k.local_position[2].y});
  for(double x:k.derivative) geometry.push_back(x);
  for(double x:k.raw_rate) geometry.push_back(x);
  for(double x:k.corrected_velocity_difference) geometry.push_back(x);
  for(double x:k.normalized_rate) geometry.push_back(x);
  ASSERT_EQ(geometry.size(),38u);
  for(unsigned i=0;i<38;++i) {
    SCOPED_TRACE("geometry "+std::to_string(i));
    Close(geometry[i],b.output[i],2e-11);
  }
  unsigned i=75;
  for(const auto& f:a.internal_force) {
    Close(f.x,b.output[i++],2e-9);
    Close(f.y,b.output[i++],2e-9);
    Close(f.z,b.output[i++],2e-9);
  }
  for(const auto& f:a.internal_couple) {
    Close(f.x,b.output[i++]);
    Close(f.y,b.output[i++]);
    Close(f.z,b.output[i++]);
  }
  const auto& d=a.diagnostics;
  const double diagnostics[]{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,
      d.shear_factor,d.transverse_shear_modulus,d.translational_stiffness,d.rotational_stiffness,
      d.unscaled_element_dt,d.internal_work_increment[0],d.internal_work_increment[1]};
  for(double x:diagnostics) Close(x,b.output[i++]);
  const auto& p=a.point.current;
  for(unsigned c=0;c<5;++c) Close(p.history.stress[c],b.output[103+c],1e-6);
  Close(p.history.plastic_strain,b.output[108]);
  Close(p.plastic_increment,b.output[109]);
  Close(p.tangent_ratio,b.output[110]);
  Close(a.point.reported_thickness_m,b.output[111],2e-15);
  Close(p.yield_before_pa,b.output[112],1e-6);
  Close(a.diagnostics.native_sound_speed,b.output[113]);
  Close(p.history.filtered_rate_per_s,b.output[115],2e-10);
  for(unsigned c=0;c<8;++c) Close(a.strain_curvature_increment[c],b.output[116+c]);
  Close(a.plastic_work_increment_j,b.output[124]);
  EXPECT_EQ(a.removed_now,b.output[125]!=0);
}
} // namespace t3_one_point_test
