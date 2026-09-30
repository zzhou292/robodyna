// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CallerTypes.h"
#include "PointChecks.h"
namespace tl::material::law90::caller_detail {
TL_LAW90_HD inline bool Positive(double x) noexcept { return tl::math::Finite(x)&&x>0; }
TL_LAW90_HD inline bool ValidHistory(const PreparedMaterial& material,
    const CallerHistory& history) noexcept {
  if (point_detail::CheckHistory(material,history.point)!=PointStatus::Ok ||
      !Positive(history.density_kg_m3) ||
      !tl::math::Finite(history.internal_energy_density_j_m3) ||
      !tl::math::Finite(history.bulk_pressure_pa) || history.bulk_pressure_pa<0 ||
      !tl::math::Finite(history.scalar_rate_per_s) || history.scalar_rate_per_s<0) return false;
  for (double x:history.stress_pa) if (!tl::math::Finite(x)) return false;
  return true;
}
TL_LAW90_HD inline bool ValidInput(const CallerInput& input,bool initialization) noexcept {
  if (!Positive(input.current_volume_m3) || !Positive(input.storage_volume_m3) ||
      !Positive(input.characteristic_length_m) ||
      !(initialization ? input.dt_s==0&&input.endpoint_time_s==0 :
        Positive(input.dt_s)&&Positive(input.endpoint_time_s))) return false;
  for (double x:input.selected_b_minus_identity) if (!tl::math::Finite(x)) return false;
  for (double x:input.engineering_rate_per_s) if (!tl::math::Finite(x)) return false;
  return true;
}
} // namespace tl::material::law90::caller_detail
