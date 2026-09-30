// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NewImpactBoundary.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline void Initialize(const NativeNewImpactInput& in,
    const NativeGeometryHistory& prior,NativeNewImpactResult& out) {
  out.history=prior;out.source_key=in.pair.key;out.source_local_main=in.pair.local_main;
  out.cache.key=in.pair.key;out.cache.occurrence=in.pair.occurrence;
  out.cache.local_main=in.pair.local_main;
  out.classification_product=in.pair.main_coefficient*::fabs(in.pair.secondary_coefficient);
  out.active=out.classification_product>0;
  for(auto& c:out.cache.sector)c.defined=FarDefined|PenetrationDefined|ClampedBarycentricDefined;
}
TL_MATH_HOST_DEVICE inline void ChooseNewImpact(const Profile& profile,
    const NativeNewImpactInput& in,NativeNewImpactResult& out) {
  if(!out.active)return;
  const int first=out.primary.subtriangle,second=out.opposite.subtriangle;
  const double primary=first>0?out.primary.penetration[first-1]:0.;
  const double opposite=second>0?out.opposite.penetration[second-1]:0.;
  out.scalar_defined=ImpactPenetrationDefined;
  int global_main=in.pair.key.main_segment,local_main=in.pair.local_main;
  if(primary>opposite&&first>0) {
    out.selected_side=ImpactSide::Primary;out.selected_subtriangle=first;
    out.penetration=primary;out.lb=out.projection[first-1].clamped_lb;
    out.lc=out.projection[first-1].clamped_lc;out.far=out.primary.far[first-1];
  } else if(opposite>primary&&second>0) {
    constexpr int sector_permutation[]{1,4,3,2};
    out.selected_side=ImpactSide::Opposite;out.selected_subtriangle=sector_permutation[second-1];
    global_main=in.opposite.global_main;local_main=in.opposite.local_main;
    out.penetration=opposite;out.lb=out.projection[second-1].clamped_lc;
    out.lc=out.projection[second-1].clamped_lb;out.far=out.opposite.far[second-1];
  } else return; // Native equal-side result is zero penetration and no winner.
  out.scalar_defined|=ImpactWeightsAndFarDefined;
  auto& row=out.history.row;
  if(row.selection_metric[0]<out.penetration||
     (row.selection_metric[0]==out.penetration&&-row.irtlm[0]<global_main)) {
    row.irtlm[0]=-global_main;row.irtlm[1]=out.selected_subtriangle;
    row.irtlm[2]=local_main;row.irtlm[3]=profile.local_processor;
    row.selection_metric[0]=out.penetration;out.row_replaced=true;
  }
  // GLOB22 publishes the selected occurrence independently of the row winner.
  out.cache.key.main_segment=global_main;out.cache.local_main=local_main;
  auto& cache=out.cache.sector[out.selected_subtriangle-1];
  cache.far=out.far;cache.penetration=out.penetration;cache.lb=out.lb;cache.lc=out.lc;
}
TL_MATH_HOST_DEVICE inline bool Finite(const NativeNewImpactResult& out) {
  if(!tl::math::Finite(out.classification_product)||!g::ValidRow(out.history.row))return false;
  for(unsigned i=0;i<4;++i) {
    const auto& p=out.projection[i];
    if(!tl::math::Finite(p.raw_lb)||!tl::math::Finite(p.raw_lc)||
       !tl::math::Finite(p.clamped_lb)||!tl::math::Finite(p.clamped_lc)||
       !tl::math::Finite(p.distance_squared)||!tl::math::Finite(out.primary.penetration[i])||
       !tl::math::Finite(out.opposite.penetration[i]))return false;
  }
  return tl::math::Finite(out.penetration)&&tl::math::Finite(out.lb)&&tl::math::Finite(out.lc);
}
} // namespace tlfea::contact::radioss_type25::selection::detail
