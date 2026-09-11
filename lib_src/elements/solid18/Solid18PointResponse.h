// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRHO3/MQVISCB/MMAIN: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18PointKinematics.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status BulkViscosity(const Material& material,
    double density, double volume, double length, PointObservation& result) noexcept {
  const auto& rate = result.engineering_rate_per_s;
  const double divergence = -rate[0]-rate[1]-rate[2];
  const double compression = ::fmax(0.0,divergence);
  // MQVISCB uses binary64 exponent constants under MYREAL8, not cbrt.
  const double edge = ::pow(volume,1.0/3.0);
  const double sound = material.sound_speed_m_s;
  const double cx = sound+::sqrt(0.0);
  const double qa = 1.0*1.1;
  const double qb = 1.0*.05;
  const double qaa0 = qa*qa;
  const double qaa = qaa0*compression;
  const double qx = qb*sound+edge*qaa+
      1.0*2*0.0/::fmax(1e-20,density*length)+
      (0.0+1.0*0.0)/::fmax(1e-20,material.density_kg_m3*length);
  result.bulk_pressure_pa = density*compression*edge*(qaa*edge+qb*sound);
  const double equivalent_sound = ::fmax(1e-20,qx+::sqrt(qx*qx+cx*cx));
  result.unscaled_element_dt_s = length/equivalent_sound;
  const double inverse_dt = 1.0/result.unscaled_element_dt_s;
  const double rho_dt = density*inverse_dt;
  const double volume_dt = volume*inverse_dt;
  result.raw_stiffness_n_m = rho_dt*volume_dt;
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
  const double increment = volume-(material.density_kg_m3/accepted.density_kg_m3)*storage;
  proposed.density_kg_m3 = material.density_kg_m3*(storage/volume);
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
