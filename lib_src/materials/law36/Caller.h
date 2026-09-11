// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law36/Update.h"
#include "lib_src/materials/detail/SolidCallerValues.h"
namespace tl::material::law36 {
// AMU is prepared here from actual density. Stress/rates already
// belong to the same native material frame. No frame/geometry is reconstructed.
namespace detail {
TL_LAW36_HD inline Status CallerValues(const Parameters& p,
    const CallerHistory& accepted, const Kinematics& input, const Measures& measures,
    CallerResult& output, bool initialization) noexcept {
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
  if (!initialization && input.dt_s == 0 &&
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
  const Status status = initialization ? Initialize(p,prepared,result.point) :
      Update(p,accepted.point,prepared,result.point);
  if (status != Status::Ok) return status;
  result.history = accepted;
  result.history.point = result.point.history;
  const double (&old)[6] = accepted.point.stress_pa;
  const double (&now)[6] = result.point.history.stress_pa;
  const auto work = solid_caller::NoEosInternalWork(old,now,input.engineering_rate_per_s,
      input.dt_s,result.average_volume_m3,measures.volume_increment_m3,
      measures.old_bulk_pressure_pa,measures.new_bulk_pressure_pa,
      accepted.internal_energy_density_j_m3,measures.storage_volume_m3,1e-20);
  result.internal_work_j = work.increment_j;
  result.history.internal_energy_density_j_m3 = work.energy_density_j_m3;
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
} // namespace detail
TL_LAW36_HD inline Status UpdateCaller(const Parameters& p,
    const CallerHistory& accepted, const Kinematics& input, const Measures& measures,
    CallerResult& output) noexcept {
  return detail::CallerValues(p,accepted,input,measures,output,false);
}
// Constructor-only TT0 evaluation. Its measures come from the actual initial
// element geometry; ordinary UpdateCaller retains the legacy dt0 restrictions.
TL_LAW36_HD inline Status InitializeCaller(const Parameters& p,
    const Kinematics& input, const Measures& measures, CallerResult& output) noexcept {
  if (input.dt_s != 0 || measures.old_bulk_pressure_pa != 0)
    return Status::InvalidInput;
  return detail::CallerValues(p,CallerHistory{},input,measures,output,true);
}
}  // namespace tl::material::law36
