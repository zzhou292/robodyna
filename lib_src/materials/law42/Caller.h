// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRHO3/MULAW/MQVISCB/MMAIN, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "CallerTypes.h"
#include "TotalStrain.h"
#include "Update.h"

namespace tl::material::law42 {
namespace caller_detail {
TL_LAW42_HD inline bool Positive(double value) noexcept {
  return tl::math::Finite(value) && value>0;
}
TL_LAW42_HD inline bool BulkViscosity(const Parameters& material,
    const CallerInput& input, CallerResult& result) noexcept {
  const auto& rate = input.engineering_rate_per_s;
  const double compression = ::fmax(0.0,-rate[0]-rate[1]-rate[2]);
  const double density = result.history.density_kg_m3;
  const double volume = input.current_volume_m3;
  const double length = input.characteristic_length_m;
  const double sound = result.point.sound_speed_m_s;
  const double edge = ::pow(volume,1.0/3.0);
  const double cx = sound+::sqrt(0.0);
  const double qa = 1.0*1.1, qb = 1.0*.05;
  const double qaa0 = qa*qa;
  const double qaa = qaa0*compression;
  const double qx = qb*sound+edge*qaa+
      1.0*2*result.point.material_viscosity_pa_s/::fmax(1e-20,density*length)+
      (0.0+1.0*0.0)/::fmax(1e-20,material.density_kg_m3*length);
  result.history.bulk_pressure_pa = density*compression*edge*(qaa*edge+qb*sound);
  const double equivalent_sound = ::fmax(1e-20,qx+::sqrt(qx*qx+cx*cx));
  result.unscaled_element_dt_s = length/equivalent_sound;
  const double inverse_dt = 1.0/result.unscaled_element_dt_s;
  const double rho_dt = density*inverse_dt;
  const double volume_dt = volume*inverse_dt;
  result.raw_stiffness_n_m = rho_dt*volume_dt;
  return Positive(result.unscaled_element_dt_s) && Positive(result.raw_stiffness_n_m) &&
         tl::math::Finite(result.history.bulk_pressure_pa);
}
} // namespace caller_detail

TL_LAW42_HD inline Status UpdateCaller(const Parameters& material,
    const CallerHistory& accepted, const CallerInput& input, CallerResult& output) noexcept {
  using caller_detail::Positive;
  Parameters checked;
  if (Prepare(material.mu_pa,material.poisson_ratio,material.density_kg_m3,
              material.tension_cutoff_pa,checked) != Status::Ok ||
      checked.bulk_pa != material.bulk_pa) return Status::InvalidParameters;
  if (!Positive(accepted.density_kg_m3) ||
      !tl::math::Finite(accepted.internal_energy_density_j_m3) ||
      !tl::math::Finite(accepted.bulk_pressure_pa) || !Positive(input.dt_s) ||
      !Positive(input.current_volume_m3) || !Positive(input.storage_volume_m3) ||
      !Positive(input.characteristic_length_m)) return Status::InvalidInput;
  for (double value : accepted.stress_pa) if (!tl::math::Finite(value)) return Status::InvalidInput;
  for (double value : input.engineering_rate_per_s) if (!tl::math::Finite(value)) return Status::InvalidInput;
  CallerResult next;
  next.history = accepted;
  next.volume_increment_m3 = input.current_volume_m3-
      (material.density_kg_m3/accepted.density_kg_m3)*input.storage_volume_m3;
  next.history.density_kg_m3 = material.density_kg_m3*
      (input.storage_volume_m3/input.current_volume_m3);
  next.average_volume_m3 = input.current_volume_m3-.5*next.volume_increment_m3;
  if (!Positive(next.history.density_kg_m3) || !Positive(next.average_volume_m3) ||
      !TotalStrain(input.displacement_gradient,next.total_strain)) return Status::NonfiniteResult;
  Input point;
  point.density_kg_m3 = next.history.density_kg_m3;
  for (unsigned k=0; k<6; ++k) point.total_strain[k] = next.total_strain[k];
  const Status status = Update(material,point,next.point);
  if (status != Status::Ok) return status;
  // This first caller admits active material only; a cutoff is never published
  // as an active element. The point leaf retains its complete cutoff behavior.
  if (next.point.active != 1) return Status::InvalidInput;
  for (unsigned k=0; k<6; ++k) next.history.stress_pa[k] = next.point.stress_pa[k];
  if (!caller_detail::BulkViscosity(material,input,next)) return Status::NonfiniteResult;
  const auto& old = accepted.stress_pa;
  const auto& now = next.history.stress_pa;
  const double pressure_sum = -(old[0]+now[0]+old[1]+now[1]+old[2]+now[2])*(1.0/3.0);
  double work[6];
  for (unsigned k=0; k<3; ++k)
    work[k] = input.engineering_rate_per_s[k]*(old[k]+now[k]+pressure_sum+2*0.0);
  for (unsigned k=3; k<6; ++k)
    work[k] = input.engineering_rate_per_s[k]*(old[k]+now[k]+2*0.0);
  next.internal_work_j = (next.average_volume_m3*input.dt_s*
      (work[0]+work[1]+work[2]+work[3]+work[4]+work[5]+0.0)-
      next.volume_increment_m3*(next.history.bulk_pressure_pa+accepted.bulk_pressure_pa+pressure_sum))*.5;
  const double energy = accepted.internal_energy_density_j_m3*input.storage_volume_m3+next.internal_work_j;
  // Like the qualified LAW36 caller, this is the native SI-packet convention.
  // It does not reproduce the working-mm floor on sub-1e-20 m3 cells.
  next.history.internal_energy_density_j_m3 = energy/::fmax(input.storage_volume_m3,1e-20);
  if (!tl::math::Finite(next.internal_work_j) ||
      !tl::math::Finite(next.history.internal_energy_density_j_m3)) return Status::NonfiniteResult;
  output = next;
  return Status::Ok;
}
} // namespace tl::material::law42
