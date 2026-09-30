// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SRHO3 -> SIGEPS44 -> MQVISCB -> MULAW/MMAIN work.
#pragma once
#include "Kinematics.h"
#include "lib_src/materials/detail/SolidCallerValues.h"

namespace tl::fea::solid18::law44::detail {
TL_SOLID18_HD inline double VonMises(const double (&stress)[6]) noexcept {
  const double x = stress[0]-stress[1];
  const double y = stress[1]-stress[2];
  const double z = stress[2]-stress[0];
  return ::sqrt(.5*(x*x+y*y+z*z)+3*(stress[3]*stress[3]+stress[4]*stress[4]+stress[5]*stress[5]));
}
TL_SOLID18_HD inline Status PointResponse(const Material& material, const PointHistory& accepted,
    const PointDerivatives& geometry, const Vec3 (&velocity)[8], double center_divergence,
    unsigned degeneracy, double dt, double length, PointHistory& proposed,
    PointObservation& observation, bool initialization = false) noexcept {
  proposed = accepted;
  const auto kinematic_status = PointKinematics(geometry,velocity,center_divergence,degeneracy,
                                               dt,proposed,observation);
  if (kinematic_status != Status::Success) return kinematic_status;
  const double volume = geometry.current_volume_m3;
  const double storage = proposed.storage_volume_m3;
  const auto density = tl::material::solid_caller::LagrangianDensity(
      material.material.density_kg_m3,accepted.density_kg_m3,storage,volume);
  proposed.density_kg_m3 = density.density_kg_m3;
  observation.volume_increment_m3 = density.volume_increment_m3;
  observation.relative_density = density.density_kg_m3/material.material.density_kg_m3-1;
  observation.average_volume_m3 = volume-.5*density.volume_increment_m3;
  if (!solid18::detail::Positive(density.density_kg_m3) ||
      !solid18::detail::Positive(observation.average_volume_m3) ||
      !tl::math::Finite(density.volume_increment_m3)) return Status::NonfiniteResult;
  point::Input input;
  input.dt_s = dt;
  input.relative_density = observation.relative_density;
  for (unsigned k = 0; k < 6; ++k) input.engineering_rate_per_s[k] = observation.engineering_rate_per_s[k];
  const auto material_status = initialization ?
      point::Initialize(material,input,observation.material) :
      point::Update(material,accepted.material,input,observation.material);
  if (material_status != point::Status::Ok) return Status::NonfiniteResult;
  proposed.material = observation.material.history;
  // MULAW uses the rounded stored PLA difference, not SIGEPS44's DPLA workspace.
  const double plastic_increment = proposed.material.plastic_strain-accepted.material.plastic_strain;
  observation.plastic_work_increment_j = .5*(VonMises(accepted.material.stress_pa)+
      VonMises(proposed.material.stress_pa))*plastic_increment*volume;
  proposed.plastic_work_j += observation.plastic_work_increment_j;
  const bool working_mm = material.material.native_units == point::WorkingUnits::TonneMillimetreSecond;
  const double density_length_floor = working_mm ? 1e-20*1e9 : 1e-20;
  const double speed_floor = working_mm ? 1e-20*.001 : 1e-20;
  const double volume_floor = working_mm ? 1e-20*(.001*.001*.001) : 1e-20;
  const auto viscosity = tl::material::solid_caller::BulkViscosity(
      input.engineering_rate_per_s,density.density_kg_m3,material.material.density_kg_m3,
      volume,length,observation.material.sound_speed_m_s,density_length_floor,speed_floor);
  observation.bulk_pressure_pa = viscosity.pressure_pa;
  observation.unscaled_element_dt_s = viscosity.unscaled_dt_s;
  observation.raw_stiffness_n_m = viscosity.stiffness_n_m;
  const auto work = tl::material::solid_caller::NoEosInternalWork(
      accepted.material.stress_pa,proposed.material.stress_pa,input.engineering_rate_per_s,dt,
      observation.average_volume_m3,density.volume_increment_m3,accepted.bulk_pressure_pa,
      viscosity.pressure_pa,proposed.energy_density_j_m3,storage,volume_floor);
  observation.internal_work_j = work.increment_j;
  proposed.energy_density_j_m3 = work.energy_density_j_m3;
  proposed.bulk_pressure_pa = viscosity.pressure_pa;
  if (!solid18::detail::Positive(viscosity.unscaled_dt_s) ||
      !solid18::detail::Positive(viscosity.stiffness_n_m) ||
      !tl::math::Finite(viscosity.pressure_pa) || !tl::math::Finite(work.increment_j) ||
      !tl::math::Finite(work.energy_density_j_m3) || !tl::math::Finite(proposed.plastic_work_j))
    return Status::NonfiniteResult;
  return Status::Success;
}
}  // namespace tl::fea::solid18::law44::detail
