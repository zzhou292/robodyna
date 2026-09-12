// SPDX-License-Identifier: AGPL-3.0-or-later
// MAIN_BEAM18/MULAW_IB, OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "ForceKinematics.h"

namespace tl::fea::beam18::force_detail {
TL_BEAM18_HD inline double Equivalent(const point::History& h) noexcept {
  const auto& s = h.stress_pa;
  return ::sqrt(s[0]*s[0]+3.*(s[1]*s[1]+s[2]*s[2]));
}
TL_BEAM18_HD inline bool SectionResponse(const Section& section, const Material& material,
    const HistoryValues& accepted, const GeneralizedRate& rate, double dt, double length,
    ForceTrial& trial, HistoryValues& next) noexcept {
  next = accepted;
  const double alpha = ::fmin(1.,material.angular_cutoff_per_s*dt);
  const double rate_norm = ::sqrt(rate.axial*rate.axial+
      .5*(rate.shear_y*rate.shear_y+rate.shear_z*rate.shear_z));
  next.filtered_neutral_rate_per_s = alpha*rate_norm+(1.-alpha)*accepted.filtered_neutral_rate_per_s;
  const double exx = rate.axial*dt, exy = rate.shear_y*dt, exz = rate.shear_z*dt;
  const double kxx = rate.curvature_x*dt, kyy = rate.curvature_y*dt, kzz = rate.curvature_z*dt;
  double membrane = accepted.section_force_n.x*exx;
  double shear = accepted.section_force_n.y*exy+accepted.section_force_n.z*exz;
  double flexure = accepted.section_moment_nm.x*kxx+accepted.section_moment_nm.y*kyy+
      accepted.section_moment_nm.z*kzz;
  next.section_force_n = {}; next.section_moment_nm = {};
  for (unsigned p = 0; p < 4; ++p) {
    const auto& at = section.point[p];
    point::Input input;
    input.strain_increment[0] = exx-at.y*kzz+at.z*kyy;
    input.strain_increment[1] = exy+at.z*kxx*1.;
    input.strain_increment[2] = exz-at.y*kxx*1.;
    input.strain_increment[1] = input.strain_increment[1]/(5./6.);
    input.strain_increment[2] = input.strain_increment[2]/(5./6.);
    for (unsigned k = 0; k < 3; ++k) next.total_strain[p][k] += input.strain_increment[k];
    input.total_axial_strain = next.total_strain[p][0];
    input.filtered_neutral_rate_per_s = next.filtered_neutral_rate_per_s;
    if (point::Update(material,accepted.point[p],input,trial.point[p]) != point::Status::Ok) return false;
    next.point[p] = trial.point[p].history;
  }
  // Native WPLA uses rounded accepted/new PLA, independently of DPLA output.
  for (unsigned p = 0; p < 4; ++p) {
    const double dpla = next.point[p].plastic_strain-accepted.point[p].plastic_strain;
    next.plastic_work_j = next.plastic_work_j +
        .5*(Equivalent(next.point[p])+Equivalent(accepted.point[p]))*dpla*length*section.area/4.;
  }
  for (unsigned p = 0; p < 4; ++p) {
    const auto& at = section.point[p]; const auto& stress = next.point[p].stress_pa;
    const double fx = at.area*stress[0], fy = at.area*stress[1], fz = at.area*stress[2];
    auto& f = next.section_force_n; auto& m = next.section_moment_nm;
    f.x = f.x+fx; f.y = f.y+fy; f.z = f.z+fz;
    m.x = m.x+fy*at.z-fz*at.y;
    m.y = m.y+fx*at.z;
    m.z = m.z-fx*at.y;
  }
  membrane = membrane+next.section_force_n.x*exx;
  shear = shear+next.section_force_n.y*exy+next.section_force_n.z*exz;
  flexure = flexure+next.section_moment_nm.x*kxx+next.section_moment_nm.y*kyy+
      next.section_moment_nm.z*kzz;
  const double factor = .5*1.*length;
  next.internal_energy_j[0] = accepted.internal_energy_j[0]+factor*(membrane+shear);
  next.internal_energy_j[1] = accepted.internal_energy_j[1]+factor*flexure;
  auto& d = trial.diagnostics;
  d.internal_work_increment_j[0] = next.internal_energy_j[0]-accepted.internal_energy_j[0];
  d.internal_work_increment_j[1] = next.internal_energy_j[1]-accepted.internal_energy_j[1];
  d.plastic_work_increment_j = next.plastic_work_j-accepted.plastic_work_j;
  return HistoryValid(material,next);
}
} // namespace tl::fea::beam18::force_detail
