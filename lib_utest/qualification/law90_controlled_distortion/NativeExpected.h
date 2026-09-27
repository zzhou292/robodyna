// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
extern "C" void law90_control_parameters_native(const double*,const double*,const double*,double*,int*);
extern "C" void distortion_force_native(int,const double*,const int*,const double*,const double*,const double*,const double*,double*,int*);
extern "C" void law90_reference_native(const double*,const double*,double*,int*,int*,int*);
extern "C" void law90_modulus_begin();
extern "C" void law90_modulus_read(double*);
namespace law90_control_test {
// Independent native material/geometry recurrence; no C++ force or parameter output enters this oracle.
struct NativeState {
  law90_force_test::NativeForce force;
  double distortion_energy=0;
  std::array<double,40> modulus{};
  explicit NativeState(double density):force(density){}
};
inline c::Result Native(const double* material,law::CurveView curve,const s::ReferenceInput& source,
    const s::PrescribedInterval& interval,d::UnitScale units,bool initial,NativeState& state) {
  const auto f=Factors(units);const auto input=WorkingReference(source,units);
  const auto step=WorkingInterval(interval,units);
  law90_modulus_begin();
  law90_force_test::AdvanceNative(material,curve,input,step,initial,state.force);
  EXPECT_EQ(state.force.status,0);
  double ordered[40];law90_modulus_read(ordered);
  const unsigned visit[]{0,4,2,6,1,5,3,7};
  for(unsigned i=0;i<8;++i)std::copy_n(ordered+5*i,5,state.modulus.data()+5*visit[i]);
  double initial_x[24],geometry[755]{};int permutation[8]{},tags[4]{},status=-1;
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)initial_x[3*n+k]=b::Component(input.position_m[n],k);
  law90_reference_native(initial_x,&input.density_kg_m3,geometry,permutation,tags,&status);EXPECT_EQ(status,0);
  const auto& values=state.force.values;
  double parameters[10]{},x[48]{},v[48]{},base[52]{},result[52]{},zero_work[52]{};int flags[2]{},contacts[2]{};
  const double kin[]{values[326],values[306],values[317]}; // G_RHO, final CXX and VOLN.
  law90_control_parameters_native(material,values.data()+320,kin,parameters,flags);
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k) {
    const unsigned source_node=permutation[n];EXPECT_LT(source_node,8u);
    x[3*n+k]=b::Component(step.position_endpoint_m[source_node],k);
    v[3*n+k]=b::Component(step.velocity_midpoint_m_s[source_node],k);
    base[3*n+k]=values[333+3*source_node+k];
  }
  base[24]=values[316];base[25]=state.distortion_energy;
  distortion_force_native(1,parameters,flags,x,v,base,&step.dt_s,result,contacts);
  base[25]=0;distortion_force_native(1,parameters,flags,x,v,base,&step.dt_s,zero_work,contacts);
  state.distortion_energy=result[25];c::Result out;
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)
    b::SetComponent(out.rhs_force_n[permutation[n]],k,result[3*n+k]*f.base.force);
  out.nodal_raw_stiffness_n_m=values[331]*f.base.stiffness;
  out.last_point_raw_stiffness_after_distortion_n_m=result[24]*f.base.stiffness;
  out.distortion_energy_j=result[25]*f.base.energy;
  out.distortion_work_increment_j=zero_work[25]*f.base.energy;
  return out;
}
} // namespace law90_control_test
