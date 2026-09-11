// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRHO3/MMAIN/MULAW/SIGEPS90, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "CallerChecks.h"
#include "Point.h"
#include "lib_src/materials/detail/SolidCallerValues.h"

namespace tl::material::law90 {
namespace caller_detail {
TL_LAW90_HD inline PointStatus Values(const PreparedMaterial& material,
    const CallerHistory& accepted,const CallerInput& input,CallerResult& output,
    bool initialization) noexcept {
  if (!material.initialized()) return PointStatus::InvalidMaterial;
  if (!ValidInput(input,initialization)) return PointStatus::InvalidInput;
  if (!initialization&&!ValidHistory(material,accepted)) return PointStatus::InvalidHistory;
  CallerResult next;
  // SRHO3 uses PM1. HM_READ_MAT also defaults MAT_PARAM%rho to PM1.
  const double rho0=material.reader().reference_density_kg_m3;
  const auto density=solid_caller::LagrangianDensity(rho0,accepted.density_kg_m3,
      input.storage_volume_m3,input.current_volume_m3);
  next.volume_increment_m3=density.volume_increment_m3;
  next.history.density_kg_m3=density.density_kg_m3;
  next.average_volume_m3=input.current_volume_m3-.5*next.volume_increment_m3;
  next.density_compression=next.history.density_kg_m3/rho0-1.0;
  if (!Positive(next.history.density_kg_m3)||!Positive(next.average_volume_m3)||
      !tl::math::Finite(next.density_compression)) return PointStatus::NonfiniteResult;
  PointKinematics kinematics;
  for (unsigned k=0;k<6;++k) {
    kinematics.total_b_minus_i_engineering[k]=input.selected_b_minus_identity[k]*1.0;
    kinematics.engineering_rate_s_inverse[k]=input.engineering_rate_per_s[k];
  }
  // MULAW ISELECT1 consumes ETOTSH and doubles the three tensor shears once.
  for (unsigned k=3;k<6;++k) kinematics.total_b_minus_i_engineering[k]*=2.0;
  const auto status=initialization ? InitializePointSI(material,kinematics,next.point) :
      UpdatePointSI(material,accepted.point,kinematics,input.endpoint_time_s,next.point);
  if (status!=PointStatus::Ok) return status;
  if (next.point.active!=1||next.point.maximum_viscosity_pa_s!=0)
    return PointStatus::InvalidInput;
  next.history.point=next.point.history;
  next.history.scalar_rate_per_s=next.point.scalar_rate_s_inverse;
  for (unsigned k=0;k<6;++k) next.history.stress_pa[k]=next.point.cauchy_stress_pa[k];
  const auto viscosity=solid_caller::BulkViscosity(input.engineering_rate_per_s,
      next.history.density_kg_m3,next.history.density_kg_m3,input.current_volume_m3,input.characteristic_length_m,
      next.point.sound_speed_m_s,1e-20,1e-20);
  next.history.bulk_pressure_pa=viscosity.pressure_pa;
  next.unscaled_element_dt_s=viscosity.unscaled_dt_s;
  next.raw_stiffness_n_m=viscosity.stiffness_n_m;
  // Native SI-packet floor; sub-floor volumes do not assert working-mm parity.
  const auto work=solid_caller::NoEosInternalWork(accepted.stress_pa,next.history.stress_pa,
      input.engineering_rate_per_s,input.dt_s,next.average_volume_m3,next.volume_increment_m3,
      accepted.bulk_pressure_pa,next.history.bulk_pressure_pa,
      accepted.internal_energy_density_j_m3,input.storage_volume_m3,1e-20);
  next.internal_work_j=work.increment_j;
  next.history.internal_energy_density_j_m3=work.energy_density_j_m3;
  if (!ValidHistory(material,next.history)||!Positive(next.unscaled_element_dt_s)||
      !Positive(next.raw_stiffness_n_m)||!tl::math::Finite(next.internal_work_j))
    return PointStatus::NonfiniteResult;
  output=next;
  return PointStatus::Ok;
}
} // namespace caller_detail
TL_LAW90_HD inline PointStatus UpdateCallerSI(const PreparedMaterial& material,
    const CallerHistory& accepted,const CallerInput& input,CallerResult& output) noexcept {
  return caller_detail::Values(material,accepted,input,output,false);
}
// Native TIME0 constructor. No accepted history or completed interval exists.
TL_LAW90_HD inline PointStatus InitializeCallerSI(const PreparedMaterial& material,
    const CallerInput& input,CallerResult& output) noexcept {
  CallerHistory virgin;
  virgin.density_kg_m3=material.reader().reference_density_kg_m3;
  return caller_detail::Values(material,virgin,input,output,true);
}
} // namespace tl::material::law90
