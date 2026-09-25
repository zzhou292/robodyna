// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ContinuationTypes.h"
#include "ClassificationValues.h"
namespace tlfea::contact::radioss_type25::selection::detail {
inline constexpr double onep02 = 1. + 2. / 100.;
TL_MATH_HOST_DEVICE inline bool Valid(const NativeContinuationInput& in,
    const NativeGeometryHistory& prior) {
  if (!Valid(in.pair,prior) || in.segment_count <= 0 ||
      in.pair.local_main > in.segment_count || in.pair.segment_type < 0 ||
      std::int64_t(in.pair.segment_type) > 2*std::int64_t(in.segment_count) ||
      in.secondary_constraint < 0 || in.secondary_skew < 0) return false;
  for(unsigned i=0;i<4;++i) {
    // COR21 reads LBOUND(ADMSR). Its authentic local reference is positive;
    // the zero/nonzero ISLIDE entry is a separate source-owned relation.
    if(in.normal_reference[i]<=0 || in.sliding_reference[i]<0 ||
       in.main_constraint[i]<0 || in.main_skew[i]<0 ||
       (in.pair.boundary_ids[i] &&
        in.pair.boundary_ids[i]!=std::uint64_t(in.normal_reference[i]))) return false;
    for(unsigned j=0;j<i;++j) {
      if(in.normal_reference[i]==in.normal_reference[j] &&
         bool(in.pair.boundary_ids[i])!=bool(in.pair.boundary_ids[j])) return false;
      if(in.pair.main_node_ids[i]==in.pair.main_node_ids[j] &&
         (in.main_constraint[i]!=in.main_constraint[j] ||
          in.main_skew[i]!=in.main_skew[j])) return false;
    }
  }
  return true;
}
TL_MATH_HOST_DEVICE inline void Initialize(const NativeContinuationInput& in,
    const NativeGeometryHistory& prior,NativeContinuationResult& out) {
  out.history=prior;out.cache.key=in.pair.key;
  out.cache.occurrence=in.pair.occurrence;out.cache.local_main=in.pair.local_main;
  out.classification_product=in.pair.main_coefficient*::fabs(in.pair.secondary_coefficient);
  out.active=out.classification_product>0;out.distance_squared=native_constant::ep20;
  for(auto& s:out.sector) {
    s.penetration=native_constant::ep20;s.distance_squared=native_constant::ep20;
    s.defined=FarDefined|PenetrationDefined|DistanceSquaredDefined;
  }
  // Each ISLIDE entry marks only the FIRST matching main reference.
  for(unsigned j=0;j<4;++j) if(in.sliding_reference[j]!=0)
    for(unsigned k=0;k<4;++k)if(in.sliding_reference[j]==in.normal_reference[k]) {
      out.sliding_match[k]=1;break;
    }
}
TL_MATH_HOST_DEVICE inline bool NativeAxisConstraint(int code,unsigned bit) {
  return code>=1 && code<=7 && (unsigned(code)&bit)!=0;
}
TL_MATH_HOST_DEVICE inline void ConstrainContinuation(const NativeContinuationInput& in,
    const Work& w,NativeContinuationResult& out) {
  if(in.pair.segment_type!=0 && in.pair.segment_type<=in.segment_count) return;
  if(in.secondary_skew==1) {
    // Native ICODT uses bits z=1,y=2,x=4. ISKM is loaded but does not enter
    // this selected source condition; do not strengthen it to a new law.
    for(unsigned bit=1;bit<=4;bit*=2) {
      bool common=NativeAxisConstraint(in.secondary_constraint,bit);
      for(unsigned i=0;i<4;++i)common=common&&NativeAxisConstraint(in.main_constraint[i],bit);
      if(common)out.constrained_axis_mask|=(bit==1?4u:bit==4?1u:2u);
    }
  }
  if(out.selected_subtriangle==0)return;
  const unsigned i=unsigned(out.selected_subtriangle-1);
  auto& s=out.sector[i];
  if(s.penetration==0)return;
  const auto normal=w.normal[i];const double precision=1.-g::em04;
  if(((out.constrained_axis_mask&4u)&&::fabs(normal.z)>precision)||
     ((out.constrained_axis_mask&2u)&&::fabs(normal.y)>precision)||
     ((out.constrained_axis_mask&1u)&&::fabs(normal.x)>precision))s.penetration=0;
}
} // namespace tlfea::contact::radioss_type25::selection::detail
