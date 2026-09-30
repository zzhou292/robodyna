// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeExpected.h"
namespace law90_control_test {
// Decode only previously accepted native state. Never seed current constitutive
// inputs, forces, coefficients or control flags in the CUDA trial being tested.
inline f::HistoryValues NativeHistory(const NativeState& state) {
  f::HistoryValues out;
  for(unsigned ip=0;ip<8;++ip) {
    const auto* raw=state.force.history+20*ip;auto& p=out.point[ip];auto& h=p.point;
    h.stress_norm_pa=raw[0];h.maximum_path_energy_pa=raw[1];h.scalar_rate_s_inverse=raw[2];
    h.path_energy_pa=raw[3];h.reserved5=raw[4];h.strain_norm=raw[5];h.unloading_factor=raw[6];
    h.effective_modulus_pa=raw[7];h.instantaneous_quasistatic_energy_pa=raw[8];h.residual_strain=raw[9];
    for(unsigned k=0;k<3;++k)h.cursor[k]=state.force.cursor[3*ip+k];
    for(unsigned k=0;k<6;++k)p.stress_pa[k]=raw[10+k];
    p.density_kg_m3=raw[16];p.internal_energy_density_j_m3=raw[17];
    p.bulk_pressure_pa=raw[18];p.scalar_rate_per_s=raw[19];
  }
  for(unsigned k=0;k<6;++k)out.global.stress_pa[k]=state.force.values[320+k];
  out.global.density_kg_m3=state.force.values[326];
  out.global.internal_energy_density_j_m3=state.force.values[327];
  out.global.bulk_pressure_pa=state.force.values[328];out.global.scalar_rate_per_s=state.force.values[329];
  return out;
}
inline void CheckNativeHistory(const c::Result& actual,const f::HistoryValues& expected) {
  const auto& values=actual.proposed_history.native_history().data();
  for(unsigned ip=0;ip<8;++ip) {
    double a[20],e[20];law90_force_test::PackHistory(values.point[ip],a);
    law90_force_test::PackHistory(expected.point[ip],e);
    for(unsigned k=0;k<20;++k)EXPECT_TRUE(law90_force_test::CheckField(a[k],e[k],law90_force_test::CallerScale(e,k),k));
    for(unsigned k=0;k<3;++k)EXPECT_EQ(values.point[ip].point.cursor[k],expected.point[ip].point.cursor[k]);
  }
  double stress_scale=1;
  for(unsigned k=0;k<6;++k)stress_scale=::fmax(stress_scale,::fabs(expected.global.stress_pa[k]));
  for(unsigned k=0;k<6;++k)EXPECT_NEAR(values.global.stress_pa[k],expected.global.stress_pa[k],2e-10*stress_scale);
  const double a[]{values.global.density_kg_m3,values.global.internal_energy_density_j_m3,
    values.global.bulk_pressure_pa,values.global.scalar_rate_per_s};
  const double e[]{expected.global.density_kg_m3,expected.global.internal_energy_density_j_m3,
    expected.global.bulk_pressure_pa,expected.global.scalar_rate_per_s};
  for(unsigned k=0;k<4;++k)EXPECT_NEAR(a[k],e[k],2e-10*::fmax(1.,::fabs(e[k])));
}
} // namespace law90_control_test
