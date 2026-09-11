// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Reference.h"

namespace tl::fea::type45::detail {
TL_TYPE45_HD inline bool Harmonic(double a, double b, double floor, double& result) {
  const double sum=a+b, product=a*b;
  if (!Finite(sum) || !Finite(product)) return false;
  result=sum!=0 ? product/::fmax(floor,sum) : 0;
  return Nonnegative(result);
}

// RUSER33 scalar/no-stop branch, including the distinction between critical
// damping on blocked DOFs and explicitly supplied viscosity on free DOFs.
TL_TYPE45_HD inline bool Response(const Reference& reference,
    const HistoryValues& accepted, HistoryValues& next, Diagnostics& diagnostics) {
  const auto& p=reference.property();
  double mass=0, inertia=0;
  if (!Harmonic(reference.damping(0).mass_kg,reference.damping(1).mass_kg,0,mass) ||
      !Harmonic(reference.damping(0).mean_principal_inertia_kg_m2,
                reference.damping(1).mean_principal_inertia_kg_m2,0,inertia)) return false;
  diagnostics.harmonic_mass_kg=mass;
  diagnostics.harmonic_inertia_kg_m2=inertia;
  DofValues displacement{next.local_displacement_m,next.relative_rotation_rad};
  DofValues rate{diagnostics.local_velocity_m_s,diagnostics.relative_rate_rad_s};
  DofValues forces;
  double max_k[2]{};
  for (unsigned i=0; i<6; ++i) {
    const double k=Get(reference.stiffness(),i);
    max_k[i/3]=::fmax(max_k[i/3],k);
    const double measure=i<3 ? mass : inertia;
    const double critical=Blocked(p.kind,i) ? p.critical_damping_ratio : 0;
    double force=k*Get(displacement,i);
    force=force+critical*::sqrt(k*measure)*Get(rate,i);
    const double viscosity=Get(p.free_viscosity,i);
    if (viscosity!=0) force=force+viscosity*Get(rate,i);
    if (!Finite(force)) return false;
    Set(forces,i,force);
  }
  double damping=p.critical_damping_ratio*::sqrt(max_k[0]*mass);
  double rotational_damping=p.critical_damping_ratio*::sqrt(max_k[1]*inertia);
  if (!Finite(damping) || !Finite(rotational_damping)) return false;
  for (unsigned i=0; i<3; ++i) {
    damping=::fmax(damping,Get(p.free_viscosity.translation,i));
    rotational_damping=::fmax(rotational_damping,Get(p.free_viscosity.rotation,i));
  }
  diagnostics.maximum_stiffness_n_m=max_k[0];
  diagnostics.maximum_rotational_stiffness_nm=max_k[1];
  diagnostics.damping_n_s_m=damping;
  diagnostics.rotational_damping_nm_s=rotational_damping;
  const DofValues old_displacement{accepted.local_displacement_m,accepted.relative_rotation_rad};
  const DofValues old_force{accepted.local_force_n,accepted.local_couple_nm};
  double work=0;
  for (unsigned i=0; i<6; ++i) {
    const double term=(Get(displacement,i)-Get(old_displacement,i))*
      (Get(forces,i)+Get(old_force,i));
    work=i==0 ? term : work+term;
  }
  diagnostics.internal_work_increment_j=.5*work;
  next.internal_work_j=accepted.internal_work_j+.5*work;
  next.local_force_n=forces.translation;
  next.local_couple_nm=forces.rotation;
  return Finite(next.internal_work_j) && Finite(diagnostics.internal_work_increment_j);
}
} // namespace tl::fea::type45::detail
