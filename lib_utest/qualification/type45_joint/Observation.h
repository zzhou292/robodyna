// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type45/Type45Force.h"

namespace type45_test {
// Named test observations only; no object padding and no persistent format.
TL_TYPE45_HD inline void Observe(const tl::fea::type45::Evaluation& value,double (&out)[55]) {
  const auto& h=value.history.values();
  unsigned index=0;
  for (double x:h.frame.v) out[index++]=x;
  const tl::math::Vec3 history_vectors[]{h.local_displacement_m,h.relative_rotation_rad,h.local_force_n,h.local_couple_nm};
  for (const auto v:history_vectors) {
    out[index++]=v.x;
    out[index++]=v.y;
    out[index++]=v.z;
  }
  out[index++]=h.internal_work_j;
  for (const auto& e:value.endpoint) {
    out[index++]=e.force_n.x;
    out[index++]=e.force_n.y;
    out[index++]=e.force_n.z;
    out[index++]=e.couple_nm.x;
    out[index++]=e.couple_nm.y;
    out[index++]=e.couple_nm.z;
    out[index++]=e.translational_stiffness_n_m;
    out[index++]=e.rotational_stiffness_nm;
  }
  const auto& d=value.diagnostics;
  const tl::math::Vec3 diagnostic_vectors[]{d.local_separation_m,d.local_velocity_m_s,d.relative_rate_rad_s};
  for (const auto v:diagnostic_vectors) {
    out[index++]=v.x;
    out[index++]=v.y;
    out[index++]=v.z;
  }
  out[index++]=d.harmonic_mass_kg;
  out[index++]=d.harmonic_inertia_kg_m2;
  out[index++]=d.maximum_stiffness_n_m;
  out[index++]=d.maximum_rotational_stiffness_nm;
  out[index++]=d.damping_n_s_m;
  out[index++]=d.rotational_damping_nm_s;
  out[index++]=d.internal_work_increment_j;
  out[index++]=value.history.stamp().time_s;
}
TL_TYPE45_HD inline bool EqualObservation(const tl::fea::type45::Evaluation& a,
                                         const tl::fea::type45::Evaluation& b) {
  if(a.history.ready()!=b.history.ready() ||
     a.history.stamp().sample_index!=b.history.stamp().sample_index) return false;
  double x[55]{},y[55]{};
  Observe(a,x);
  Observe(b,y);
  for(unsigned i=0;i<55;++i) if(!tl::fea::type45::detail::Same(x[i],y[i])) return false;
  return true;
}
} // namespace type45_test
