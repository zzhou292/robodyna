// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected S6ZHOUR3/SZETFAC, with the explicit LAW42 CXX repair receipt.
#pragma once
#include "Solid6zHourglassGeometry.h"
#include "lib_src/elements/solid_common/PhysicalHourglassModes.h"

namespace tl::fea::solid6z::force_detail {
TL_BRICK_HD inline Status Stabilize(const Reference& reference, const Material& material,
    const ForceProfile& profile, const CurrentGeometry& current, double dt_s,
    const tl::material::law42::CallerResult& point, HistoryValues& history,
    Vec3 (&local_force)[6], HourglassObservation& output) noexcept {
  namespace common = tl::fea::solid_common;
  HourglassGeometry geometry;
  if (!StabilizationGeometry(current,geometry)) return Status::InvalidGeometry;
  HourglassObservation next;
  const double et = point.point.hourglass_tangent_factor;
  const double tangent_factor = et <= 1 ? ::fmax(.05,et) : .2+et;
  // HM_READ_MAT42 PARMAT1=SUM(Mu*alpha); HM_READ_MAT assigns PM22=PARMAT1.
  const double native_g0 = material.mu_pa*2;
  const double shear = tangent_factor*native_g0;
  const double caq = .25*1.0*profile.damping_coefficient;
  double damping = 1.1*caq*history.material.density_kg_m3*
      ::pow(current.current_volume_m3,1.0/3.0);
  // Corrected S6ZHOUR3 local CXX is the actual returned material SSP.
  damping = (2.0/300.0)*damping*point.point.sound_speed_m_s;
  next.effective_shear_modulus_pa = shear;
  next.damping_kg_m_s = damping;
  const double increment = (1.0/3.0)*1.0*shear*dt_s;
  const auto& diagonal = geometry.diagonal_m;
  double inverse[3];
  for (unsigned k = 0; k < 3; ++k) inverse[k] = 1.0/::fmax(1e-20,diagonal[k]);
  const double shape[6]{diagonal[1]*diagonal[2]*inverse[0],
      diagonal[0]*diagonal[2]*inverse[1],diagonal[0]*diagonal[1]*inverse[2],
      diagonal[2],diagonal[1],diagonal[0]};
  common::ModeRates(geometry.velocity_m_s,geometry.projection,next.modal_velocity_m_s);
  auto& stress = history.hourglass_stress_pa;
  double temporary[3][4];
  for (unsigned component = 0; component < 3; ++component) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      stress[component][mode] = stress[component][mode]*1.0;
      temporary[component][mode] = stress[component][mode]*diagonal[component]+
          damping*next.modal_velocity_m_s[component][mode];
    }
  }
  common::CoupledModes(shape,temporary,next.modal_force_n);
  const double storage_volume = reference.geometry().volume_m3;
  next.first_work_j = .5*dt_s*common::ModePower(next.modal_force_n,next.modal_velocity_m_s);
  double energy = history.material.internal_energy_density_j_m3+
      next.first_work_j/::fmax(1e-20,storage_volume);
  for (unsigned component = 0; component < 3; ++component) {
    const double scale = increment*inverse[component];
    for (unsigned mode = 0; mode < 4; ++mode) {
      const double change = scale*next.modal_velocity_m_s[component][mode];
      stress[component][mode] = stress[component][mode]+change;
      temporary[component][mode] = stress[component][mode]*diagonal[component]+
          damping*next.modal_velocity_m_s[component][mode];
    }
  }
  common::CoupledModes(shape,temporary,next.modal_force_n);
  Vec3 expanded_force[8];
  common::ModeForces(geometry.projection,next.modal_force_n,expanded_force);
  for (unsigned component = 0; component < 3; ++component) {
    const double f1 = common::Component(local_force[0],component)+common::Component(expanded_force[0],component);
    const double f2 = common::Component(local_force[1],component)+common::Component(expanded_force[1],component);
    const double f3 = common::Component(local_force[2],component)+common::Component(expanded_force[2],component)+
        common::Component(expanded_force[3],component);
    const double f4 = common::Component(local_force[3],component)+common::Component(expanded_force[4],component);
    const double f5 = common::Component(local_force[4],component)+common::Component(expanded_force[5],component);
    const double f6 = common::Component(local_force[5],component)+common::Component(expanded_force[6],component)+
        common::Component(expanded_force[7],component);
    common::SetComponent(local_force[0],component,f1);
    common::SetComponent(local_force[1],component,f2);
    common::SetComponent(local_force[2],component,f3);
    common::SetComponent(local_force[3],component,f4);
    common::SetComponent(local_force[4],component,f5);
    common::SetComponent(local_force[5],component,f6);
  }
  next.second_work_j = .5*dt_s*common::ModePower(next.modal_force_n,next.modal_velocity_m_s);
  energy = energy+next.second_work_j/::fmax(1e-20,storage_volume);
  if (!tl::math::Finite(energy) || !tl::math::Finite(shear) || !tl::math::Finite(damping) ||
      !tl::math::Finite(next.first_work_j) || !tl::math::Finite(next.second_work_j)) return Status::NonfiniteResult;
  for (const auto& component : stress) {
    for (double value : component) {
      if (!tl::math::Finite(value)) return Status::NonfiniteResult;
    }
  }
  for (const auto& force : local_force) {
    if (!common::Finite(force)) return Status::NonfiniteResult;
  }
  history.material.internal_energy_density_j_m3 = energy;
  output = next;
  return Status::Success;
}
} // namespace tl::fea::solid6z::force_detail
