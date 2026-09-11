// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ObserverTypes.h"
#include "../NodalWallContactReduction.h"
#include <cfloat>
namespace tlfea::contact::nodal_wall_mapped {
TL_SURFACE_HD inline bool ObserverIntegralDomain(Q4CertifiedIntegral x) noexcept {
  return IsFinite(x.value) && x.value>=0 && IsFinite(x.lower) && x.lower>=0 &&
      IsFinite(x.upper) && x.upper>=x.lower;
}
TL_SURFACE_HD inline void ObserveMagnitude(ObserverSummary& out,double value) noexcept {
  if(!IsFinite(value)) out.serial=true;
  else out.maximum_term=::fmax(out.maximum_term,::fabs(value));
}
TL_SURFACE_HD inline bool AddIntegral(ObserverIntegral& out,Q4CertifiedIntegral term) noexcept {
  auto value=out.Get();
  if(!nodal_wall_reduction::Sum(value,term)) return false;
  out.Set(value);return true;
}
TL_SURFACE_HD inline void ObserveNode(const NodalWallPointResult& node,double penetration,
    ObserverSummary& out) noexcept {
  // Radius is recomputed by Sum from the consumed nominal/lower/upper fields;
  // the input error field itself is not read by the original global fold.
  const double values[]={node.force.value,node.force.lower,node.force.upper,
    node.potential.value,node.potential.lower,node.potential.upper,
    node.wall_reaction.x,node.wall_reaction.y,node.wall_reaction.z,
    node.wall_moment.x,node.wall_moment.y,node.wall_moment.z,node.surface_power};
  for(double value:values)ObserveMagnitude(out,value);
  if(!ObserverIntegralDomain(node.force)||!ObserverIntegralDomain(node.potential))out.serial=true;
  if(!AddIntegral(out.resultant,node.force)||!AddIntegral(out.potential,node.potential)) out.serial=true;
  for(unsigned c=0;c<7;++c) {
    out.signed_sum[c]+=values[6+c];
    if(!IsFinite(out.signed_sum[c])) out.serial=true;
  }
  // Preserve the physical branch and strict comparison. Exceptional geometry
  // replays the old code, including its original treatment of inactive points.
  if(!IsFinite(penetration)) out.serial=true;
  if(node.stiffness.value>0 && penetration>out.maximum_penetration) out.maximum_penetration=penetration;
}
TL_SURFACE_HD inline void MergeObservers(ObserverSummary& a,const ObserverSummary& b) noexcept {
  a.serial=a.serial||b.serial;
  a.maximum_term=::fmax(a.maximum_term,b.maximum_term);
  if(!AddIntegral(a.resultant,b.resultant.Get())||!AddIntegral(a.potential,b.potential.Get())) a.serial=true;
  for(unsigned c=0;c<7;++c) {
    a.signed_sum[c]+=b.signed_sum[c];
    if(!IsFinite(a.signed_sum[c])) a.serial=true;
  }
  if(b.maximum_penetration>a.maximum_penetration)a.maximum_penetration=b.maximum_penetration;
}
// Sufficient proof for the old serial interval operations as well as nominal
// sums. N bounds all leaves plus the seed. For admitted N, directed additions
// grow by <1e-9 relative; the factor32 margin also covers two-sum's subtraction
// intermediates, certificate endpoint differences and their outward rounding.
// Failure selects serial evaluation, never a new admission rejection.
TL_SURFACE_HD inline bool FiniteObserverPrefixes(std::size_t nodes,double maximum) noexcept {
  return ObserverBlocks(nodes) && IsFinite(maximum) && maximum>=0 &&
      maximum<=DBL_MAX/(32.0*(static_cast<double>(nodes)+1));
}
TL_SURFACE_HD inline bool ApplyObservers(const ObserverSummary& summary,std::size_t nodes,
    NodalWallDiagnostics& diagnostics) noexcept {
  if(summary.serial) return false;
  if(!ObserverIntegralDomain(diagnostics.resultant)||!ObserverIntegralDomain(diagnostics.potential))return false;
  ObserverSummary next{};
  next.resultant.Set(diagnostics.resultant);next.potential.Set(diagnostics.potential);
  const double seeds[]={diagnostics.resultant.value,diagnostics.resultant.lower,diagnostics.resultant.upper,
    diagnostics.potential.value,diagnostics.potential.lower,diagnostics.potential.upper,
    diagnostics.wall_reaction.x,diagnostics.wall_reaction.y,diagnostics.wall_reaction.z,
    diagnostics.wall_moment.x,diagnostics.wall_moment.y,diagnostics.wall_moment.z,diagnostics.surface_power};
  for(double value:seeds)ObserveMagnitude(next,value);
  for(unsigned c=0;c<7;++c)next.signed_sum[c]=seeds[6+c];
  // Production seeds zero. Keep unusual signed/negative/nonfinite seeds on the
  // legacy branch rather than altering its strict-maximum initial semantics.
  if(!IsFinite(diagnostics.maximum_penetration)||diagnostics.maximum_penetration<0) return false;
  next.maximum_penetration=diagnostics.maximum_penetration;
  MergeObservers(next,summary);
  if(next.serial || !FiniteObserverPrefixes(nodes,next.maximum_term)) return false;
  diagnostics.resultant=next.resultant.Get();diagnostics.potential=next.potential.Get();
  diagnostics.wall_reaction={next.signed_sum[0],next.signed_sum[1],next.signed_sum[2]};
  diagnostics.wall_moment={next.signed_sum[3],next.signed_sum[4],next.signed_sum[5]};
  diagnostics.surface_power=next.signed_sum[6];
  diagnostics.maximum_penetration=next.maximum_penetration;
  return true;
}
} // namespace tlfea::contact::nodal_wall_mapped
