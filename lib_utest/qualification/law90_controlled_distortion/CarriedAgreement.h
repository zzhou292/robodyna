// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Diagnostics.h"
extern "C" void law90_control_viscosity_native(const double*,const double*,const double*,const double*,const double*,const double*,double*);
namespace law90_control_test {
inline double ModulusReplay(const double* input,double& residual) {
  residual=input[0]-input[1]/input[2];residual=::fmax(0.,residual);residual=::fmin(1.,residual);
  return ::fmin(input[4],(input[4]-input[3])*residual+input[3]);
}
struct Drift {double modulus_relative=0,stiffness_relative=0;};
template<class Case>
void CheckCarried(const Case& actual,const std::vector<std::array<double,357>>& native,
    const std::vector<std::array<double,40>>& native_modulus,unsigned row,Drift& drift) {
  constexpr double u=0x1p-53;
  for(unsigned step=0;step<(Case::steps+1);++step) {
    const auto& result=actual.output[step];const auto& h=result.proposed_history.native_history();
    const auto& op=actual.operands[step];const auto factors=Factors(result.proposed_history.units());
    SCOPED_TRACE(row*(Case::steps+1)+step);
    for(unsigned ip=0;ip<8;++ip) {
      const auto& point=h.data().point[ip];const auto* gpu=op.modulus[ip];
      const auto* expected=native_modulus[row*(Case::steps+1)+step].data()+5*ip;
      const auto* expected_point=native[row*(Case::steps+1)+step].data()+40*ip;
      double residual=0;EXPECT_EQ(ModulusReplay(gpu,residual),point.point.effective_modulus_pa);
      EXPECT_EQ(residual,point.point.residual_strain);
      double native_residual=0;EXPECT_EQ(ModulusReplay(expected,native_residual),expected_point[7]);
      EXPECT_EQ(native_residual,expected_point[9]);
      EXPECT_EQ(gpu[2],step?actual.output[step-1].proposed_history.native_history().data().point[ip].point.effective_modulus_pa:gpu[3]);
      EXPECT_EQ(gpu[3],expected[3]);EXPECT_EQ(gpu[4],expected[4]);
      // Primary norm differences retain a strict binary64/material scale. Their
      // nonlinear amplification is reported, not hidden by a trajectory tolerance.
      EXPECT_NEAR(gpu[0],expected[0],32*u*::fmax(1.,::fabs(expected[0])));
      EXPECT_NEAR(gpu[1],expected[1],32*u*::fmax(::fabs(expected[1]),expected[3]*::fmax(1.,::fabs(expected[0]))));
      for(unsigned k:{0u,1u,3u,4u,5u,6u}) {
        double packed[20];law90_force_test::PackHistory(point,packed);
        EXPECT_EQ(packed[k],expected_point[k]); // IFLAG1 genuinely carries these unchanged.
      }
      EXPECT_NEAR(point.point.scalar_rate_s_inverse,expected_point[2],2e-10*::fmax(1.,::fabs(expected_point[2])));
      EXPECT_NEAR(point.point.instantaneous_quasistatic_energy_pa,expected_point[8],2e-10*::fmax(1.,::fabs(expected_point[8])));
      for(unsigned k=0;k<3;++k)EXPECT_EQ(point.point.cursor[k],actual.native_history[step].point[ip].point.cursor[k]);
      double stress_scale=1;
      for(unsigned k=0;k<6;++k)stress_scale=::fmax(stress_scale,::fabs(expected_point[10+k]));
      for(unsigned k=0;k<6;++k)EXPECT_NEAR(point.stress_pa[k],expected_point[10+k],2e-10*stress_scale);
      const double nu=h.material().reader().poisson_ratio,rho0=h.material().reader().reference_density_kg_m3;
      const double longitudinal=point.point.effective_modulus_pa*(1-nu)/(1+nu)/(1-2*nu);
      EXPECT_EQ(op.sound[ip],::sqrt(longitudinal/rho0));
      double viscosity[3];law90_control_viscosity_native(op.rate[ip],&point.density_kg_m3,&rho0,
          &op.volume[ip],&op.length[ip],&op.sound[ip],viscosity);
      EXPECT_NEAR(op.stiffness[ip],viscosity[2],2e-10*::fabs(viscosity[2]));
      EXPECT_NEAR(point.bulk_pressure_pa,viscosity[0],2e-10*::fabs(viscosity[0]));
      drift.modulus_relative=::fmax(drift.modulus_relative,::fabs(point.point.effective_modulus_pa-expected_point[7])/expected_point[7]);
      drift.stiffness_relative=::fmax(drift.stiffness_relative,::fabs(op.stiffness[ip]-expected_point[36])/expected_point[36]);
    }
    double sum=0;for(unsigned ip:{0u,4u,2u,6u,1u,5u,3u,7u})sum=sum+op.stiffness[ip];
    EXPECT_EQ(result.nodal_raw_stiffness_n_m,sum*factors.base.stiffness);
  }
}
} // namespace law90_control_test
