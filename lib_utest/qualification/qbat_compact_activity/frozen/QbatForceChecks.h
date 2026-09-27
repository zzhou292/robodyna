// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatForceTypes.h"
#include "lib_src/materials/Law44MaterialScope.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"
#include "lib_src/elements/qeph/QephHistory.h"

namespace tl::fea::qbat::detail {
template<unsigned N> TL_QBAT_HD inline bool FiniteValues(const double (&values)[N]) {
  for (double x:values) if (!tl::math::Finite(x)) return false;
  return true;
}
TL_QBAT_HD inline bool Same(double a,double b) {
  return qeph::detail::SameHistoryBits(a,b);
}
TL_QBAT_HD inline bool SameReference(const Reference& a,const Reference& b) {
  if (!a.prepared() || !b.prepared()) return false;
  const auto& x=a.input();
  const auto& y=b.input();
  if (!Supported(x.options) || !Supported(y.options)) return false;
  const auto& q=x.quadrilateral;
  const auto& r=y.quadrilateral;
  for (unsigned i=0;i<4;++i) {
    if (q.node_ids[i]!=r.node_ids[i] || !Same(q.position[i].x,r.position[i].x) ||
        !Same(q.position[i].y,r.position[i].y) || !Same(q.position[i].z,r.position[i].z)) return false;
  }
  return q.placement==r.placement &&
      Same(q.density,r.density) && Same(q.young_modulus,r.young_modulus) &&
      Same(q.poisson_ratio,r.poisson_ratio) && Same(q.thickness,r.thickness) &&
      Same(x.initial_a11_pa,y.initial_a11_pa) &&
      Same(x.options.membrane_viscosity,y.options.membrane_viscosity) &&
      Same(x.options.numerical_viscosity,y.options.numerical_viscosity);
}
TL_QBAT_HD inline bool SameMaterial(const Material& a,const Material& b) {
  return tl::material::SameLaw44Parameters(a,b);
}
TL_QBAT_HD inline bool ValidMaterial(const Reference& r,const Material& p,Failure f) {
  return r.prepared() && Supported(r.input().options) &&
      sections::ValidLayeredJ2Parameters(p) &&
      sections::MatchesLayeredJ2Material(p,r.input().quadrilateral) &&
      Same(p.a11,r.input().initial_a11_pa) && Positive(f.failure_strain);
}
TL_QBAT_HD inline bool ValidHistory(const HistoryValues& h,const Material& p,double time) {
  if (!Positive(h.thickness_m) || !FiniteValues(h.force_stress_pa) ||
      !FiniteValues(h.strain) || !FiniteValues(h.internal_work_j) ||
      !tl::math::Finite(h.plastic_work_j) || h.plastic_work_j<0 ||
      !tl::math::Finite(h.numerical_viscous_work_j) || h.numerical_viscous_work_j<0 ||
      !tl::math::Finite(h.reported_rate_per_s) || h.reported_rate_per_s<0) return false;
  bool any_active=false;
  for (const auto& point:h.point) {
    const auto& m=point.material;
    const auto& f=point.failure;
    if (!FiniteValues(m.stress) || m.stress[3]!=0 || m.stress[4]!=0 ||
        !tl::math::Finite(m.plastic_strain) || m.plastic_strain<0 ||
        !tl::material::tabulated_shell_detail::HardeningDomain(p,m.plastic_strain) ||
        !tl::math::Finite(m.filtered_rate_per_s) || m.filtered_rate_per_s<0 ||
        (!p.rate.enabled && m.filtered_rate_per_s!=0) ||
        !FiniteValues(point.force_stress_pa) || !FiniteValues(point.strain) ||
        !tl::math::Finite(f.damage) || f.damage<0 || f.damage>1 ||
        !tl::math::Finite(f.failure_time_s) || f.failure_time_s<0 ||
        f.failure_time_s>time || point.surface_active!=f.point_active ||
        (f.point_active && (f.damage>=1 || f.failure_time_s!=0)) ||
        (!f.point_active && f.damage!=1)) return false;
    if (!f.point_active) {
      for (double s:m.stress) if (s!=0) return false;
    }
    for (unsigned i=3;i<5;++i) if (point.force_stress_pa[i]!=0) return false;
    for (unsigned i=3;i<8;++i) if (point.strain[i]!=0) return false;
    any_active=any_active || point.surface_active;
  }
  if (h.element_active!=any_active) return false;
  for (unsigned i=0;i<5;++i) {
    const double mean=.25*(h.point[0].force_stress_pa[i]+h.point[1].force_stress_pa[i]+
        h.point[2].force_stress_pa[i]+h.point[3].force_stress_pa[i]);
    if (!tl::math::Finite(mean) || h.force_stress_pa[i]!=mean) return false;
  }
  for (unsigned i=3;i<8;++i) if (h.strain[i]!=0) return false;
  return h.internal_work_j[1]==0;
}
} // namespace tl::fea::qbat::detail
