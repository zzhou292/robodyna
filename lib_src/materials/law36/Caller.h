// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law36/Update.h"
namespace tl::material::law36 {
// AMU is prepared here from actual density. Stress/rates already
// belong to the same native material frame. No frame/geometry is reconstructed.
TL_LAW36_HD inline Status UpdateCaller(const Parameters& p,
    const CallerHistory& accepted, const Kinematics& input, const Measures& measures,
    CallerResult& output) noexcept {
  if (!detail::ParametersValid(p)) return Status::InvalidParameters;
  if (!detail::Positive(measures.density_kg_m3) ||
      !detail::Positive(measures.storage_volume_m3) ||
      !detail::Positive(measures.current_volume_m3) ||
      !tl::math::Finite(measures.volume_increment_m3) ||
      !tl::math::Finite(measures.old_bulk_pressure_pa) ||
      !tl::math::Finite(measures.new_bulk_pressure_pa))
    return Status::InvalidInput;
  if (!tl::math::Finite(accepted.internal_energy_density_j_m3) ||
      !tl::math::Finite(accepted.plastic_work_j) || accepted.plastic_work_j < 0)
    return Status::InvalidHistory;
  if (input.dt_s == 0 &&
      (accepted.internal_energy_density_j_m3 != 0 || accepted.plastic_work_j != 0 ||
       measures.density_kg_m3 != p.density_kg_m3 ||
       measures.current_volume_m3 != measures.storage_volume_m3 ||
       measures.volume_increment_m3 != 0 || measures.old_bulk_pressure_pa != 0 ||
       measures.new_bulk_pressure_pa != 0)) return Status::InvalidInput;
  CallerResult result{};
  Input prepared{};
  prepared.kinematics = input;
  prepared.relative_density = measures.density_kg_m3 / p.density_kg_m3 - 1;
  result.relative_density = prepared.relative_density;
  result.average_volume_m3 = measures.current_volume_m3 - .5 * measures.volume_increment_m3;
  if (!detail::Positive(result.average_volume_m3)) return Status::InvalidInput;
  const Status status = Update(p, accepted.point, prepared, result.point);
  if (status != Status::Ok) return status;
  result.history = accepted;
  result.history.point = result.point.history;
  const double (&old)[6] = accepted.point.stress_pa;
  const double (&now)[6] = result.point.history.stress_pa;
  // MULAW: unmasked old/new stress sum, pressure removed for deviatoric work.
  const double pressure_sum =
      -(old[0]+now[0]+old[1]+now[1]+old[2]+now[2]) * (1.0 / 3.0);
  double work[6]{};
  for (unsigned i = 0; i < 3; ++i) {
    work[i] = input.engineering_rate_per_s[i] * (old[i]+now[i]+pressure_sum+2*0.0);
  }
  for (unsigned i = 3; i < 6; ++i) {
    work[i] = input.engineering_rate_per_s[i] * (old[i]+now[i]+2*0.0);
  }
  result.internal_work_j =
      (result.average_volume_m3 * input.dt_s *
       (work[0]+work[1]+work[2]+work[3]+work[4]+work[5]+0.0) -
       measures.volume_increment_m3 *
       (measures.new_bulk_pressure_pa+measures.old_bulk_pressure_pa+pressure_sum)) * .5;
  const double energy = accepted.internal_energy_density_j_m3 * measures.storage_volume_m3 +
                        result.internal_work_j;
  // MMAIN's literal floor is retained in this SI packet convention. It is not
  // a conversion of the original t/mm/s floor; see the near-floor domain note.
  result.history.internal_energy_density_j_m3 = energy / ::fmax(measures.storage_volume_m3, 1e-20);
  result.plastic_work_increment_j =
      result.point.plastic_work_density_j_m3 * measures.current_volume_m3;
  result.history.plastic_work_j += result.plastic_work_increment_j;
  if (!tl::math::Finite(result.internal_work_j) ||
      !tl::math::Finite(result.history.internal_energy_density_j_m3) ||
      !tl::math::Finite(result.plastic_work_increment_j) ||
      !tl::math::Finite(result.history.plastic_work_j)) return Status::NonfiniteResult;
  output = result;
  return Status::Ok;
}
}  // namespace tl::material::law36
