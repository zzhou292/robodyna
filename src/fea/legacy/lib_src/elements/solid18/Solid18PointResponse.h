// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRHO3/MQVISCB/MMAIN: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18PointKinematics.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status BulkViscosity(const Material& material,
    double density, double volume, double length, PointObservation& result) noexcept {
  const auto values = tl::material::solid_caller::BulkViscosity(
      result.engineering_rate_per_s,density,material.density_kg_m3,volume,length,
      material.sound_speed_m_s,1e-20,1e-20);
  result.bulk_pressure_pa = values.pressure_pa;
  result.unscaled_element_dt_s = values.unscaled_dt_s;
  result.raw_stiffness_n_m = values.stiffness_n_m;
  if (!Positive(result.unscaled_element_dt_s) || !Positive(result.raw_stiffness_n_m) ||
      !tl::math::Finite(result.bulk_pressure_pa)) return Status::NonfiniteResult;
  return Status::Success;
}

TL_SOLID18_HD inline Status PointResponse(const Material& material,
    const PointHistory& accepted, const PointDerivatives& geometry,
    const Vec3 (&velocity)[8], double dt, double length,
    PointHistory& proposed, PointObservation& observation, bool initialization = false) noexcept {
  proposed = accepted;
  Status status = PointKinematics(geometry,velocity,dt,proposed,observation);
  if (status != Status::Success) return status;
  const double volume = geometry.current_volume_m3;
  const double storage = proposed.storage_volume_m3;
  const auto density = tl::material::solid_caller::LagrangianDensity(
      material.density_kg_m3,accepted.density_kg_m3,storage,volume);
  const double increment = density.volume_increment_m3;
  proposed.density_kg_m3 = density.density_kg_m3;
  observation.volume_increment_m3 = increment;
  if (!Positive(proposed.density_kg_m3) || !tl::math::Finite(increment))
    return Status::NonfiniteResult;
  // The selected LAW36 point leaves SSP unchanged. Computing this geometry-only
  // q first permits one shared material/work call; the native oracle retains
  // MMAIN's actual point -> MQVISCB -> work order and verifies that handoff.
  status = BulkViscosity(material,proposed.density_kg_m3,volume,length,observation);
  if (status != Status::Success) return status;
  tl::material::law36::Kinematics kinematics;
  kinematics.dt_s = dt;
  for (unsigned k = 0; k < 6; ++k) {
    kinematics.engineering_rate_per_s[k] = observation.engineering_rate_per_s[k];
  }
  tl::material::law36::Measures measures;
  measures.density_kg_m3 = proposed.density_kg_m3;
  measures.storage_volume_m3 = storage;
  measures.current_volume_m3 = volume;
  measures.volume_increment_m3 = increment;
  measures.old_bulk_pressure_pa = accepted.bulk_pressure_pa;
  measures.new_bulk_pressure_pa = observation.bulk_pressure_pa;
  const auto material_status = initialization ?
      tl::material::law36::InitializeCaller(material,kinematics,measures,observation.material) :
      tl::material::law36::UpdateCaller(material,proposed.material,kinematics,measures,observation.material);
  if (material_status != tl::material::law36::Status::Ok) return Status::InvalidInput;
  proposed.material = observation.material.history;
  proposed.bulk_pressure_pa = observation.bulk_pressure_pa;
  return Status::Success;
}
}  // namespace tl::fea::solid18::detail
