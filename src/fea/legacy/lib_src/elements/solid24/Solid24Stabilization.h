// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SZHOUR3/SZETFAC, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Solid24ForceTypes.h"
#include "lib_src/elements/solid_common/PhysicalHourglassModes.h"

namespace tl::fea::solid24::force_detail {
TL_BRICK_HD inline ForceStatus Stabilization(const Reference& reference,const Material& material,
    const ForceGeometry& g,double dt,HistoryValues& history,ForceDiagnostics& diagnostics,
    Vec3 (&force)[8]) noexcept {
  const double et=diagnostics.material.point.hourglass_tangent_factor;
  const double factor=et<=1 ? ::fmax(.05,et) : .2+et;
  // HM_READ_MAT: LAW42 PM22=PARMAT1=GS, which is twice printed MU0.
  const double initial_modulus=material.mu_pa*2;
  const double shear=initial_modulus*factor;
  diagnostics.stabilization_modulus_pa=shear;
  // Public DN=.1, native CVIS. No source IHQ2/QM value enters this formula.
  const double caq=.25*1.0*.1;
  double visc=1.1*caq*history.material.density_kg_m3*::pow(g.current.volume_m3,1.0/3.0);
  visc=(2.0/300.0)*visc*diagnostics.material.point.sound_speed_m_s;
  diagnostics.stabilization_viscosity_kg_m_s=visc;
  const double increment=(1.0/3.0)*1.0*shear*dt;
  const auto& j=g.jacobian_diagonal_m;
  double inverse[3];
  for (unsigned k=0; k<3; ++k) inverse[k]=1.0/::fmax(1e-20,j[k]);
  const double shape[]{j[1]*j[2]*inverse[0],j[0]*j[2]*inverse[1],j[0]*j[1]*inverse[2],
                       j[2],j[1],j[0]};
  double rate[3][4],t[3][4],mode[3][4];
  namespace modes=tl::fea::solid_common;
  modes::ModeRates(g.local_velocity_m_s,g.hourglass_projection,rate);
  auto& h=history.physical_hourglass;
  for (unsigned k=0; k<3; ++k) for (unsigned n=0; n<4; ++n) {
    h[k][n]=h[k][n]*1.0;
    t[k][n]=h[k][n]*j[k]+visc*rate[k][n];
  }
  modes::CoupledModes(shape,t,mode);
  const double before=.5*dt*modes::ModePower(mode,rate);
  const double volume=reference.geometry().volume_m3;
  double energy=history.material.internal_energy_density_j_m3+before/::fmax(1e-20,volume);
  for (unsigned k=0; k<3; ++k) {
    const double scale=increment*inverse[k];
    for (unsigned n=0; n<4; ++n) {
      h[k][n]=h[k][n]+scale*rate[k][n];
      t[k][n]=h[k][n]*j[k]+visc*rate[k][n];
    }
  }
  modes::CoupledModes(shape,t,mode);
  modes::ModeForces(g.hourglass_projection,mode,force);
  const double after=.5*dt*modes::ModePower(mode,rate);
  energy=energy+after/::fmax(1e-20,volume);
  diagnostics.stabilization_work_j=before+after;
  if (!tl::math::Finite(energy) || !tl::math::Finite(diagnostics.stabilization_work_j) ||
      !tl::math::Finite(visc) || !tl::math::Finite(shear)) return ForceStatus::NonfiniteResult;
  for (const auto& row:h) for (double value:row)
    if (!tl::math::Finite(value)) return ForceStatus::NonfiniteResult;
  history.material.internal_energy_density_j_m3=energy;
  return ForceStatus::Success;
}
} // namespace tl::fea::solid24::force_detail
