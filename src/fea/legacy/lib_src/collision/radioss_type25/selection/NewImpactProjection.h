// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NewImpactPrepare.h"
#include "ClassificationValues.h"
namespace tlfea::contact::radioss_type25::selection::detail {
struct ImpactProjectionView {
  SectorValues (&sector)[4];
  double distance_squared=native_constant::ep20; // Private closest-sector scratch.
};
TL_MATH_HOST_DEVICE inline double ImpactGap(const NativePairInput& in,const Work& w,
    const SectorValues& s,unsigned i,bool primary_penetration=false) {
  const unsigned next=(i+1)%4;const double la=1.-s.clamped_lb-s.clamped_lc;
  const double geometric=in.secondary_gap+la*w.center_gap+s.clamped_lb*in.main_gap[i]+
      s.clamped_lc*in.main_gap[next];
  // The primary penetration loop adds DGAPLOAD after the first MAX; the
  // cylindrical-gap and opposite loops add it inside. Preserve both spellings.
  const double first=primary_penetration?
      g::Max(in.radiation_range,geometric)+in.applied_gap:
      g::Max(in.radiation_range,geometric+in.applied_gap);
  return g::Min(first,g::Max(in.radiation_range,
      native_constant::ep20*native_constant::ep10+in.applied_gap));
}
TL_MATH_HOST_DEVICE inline void ProjectNewImpact(const NativeNewImpactInput& in,
    ImpactWork& work,NativeNewImpactResult& out) {
  auto& geometry=work.geometry;ImpactProjectionView view{out.projection};
  // Unlike retained/continuation, this original work executes even if the
  // coefficient product is nonpositive, including ALL four T3 sectors.
  for(auto& s:out.projection)s.defined=DistanceSquaredDefined;
  RawProjection(geometry,view);
  for(unsigned i=0;i<4;++i) {
    auto& s=out.projection[i];
    if(s.raw_lb<-g::em03||s.raw_lc<-g::em03||s.raw_lb+s.raw_lc>1.+g::em03)
      out.primary.far[i]=out.opposite.far[i]=1;
    ProjectedSectorPoint(in.pair,geometry,view,i);
  }
  const bool keep=!(work.opposite_local>0&&out.primary.intersection==0&&
      out.opposite.intersection==0&&in.pair.initial_contact_flag<0);
  for(unsigned i=0;i<4;++i) {
    work.gap[i]=ImpactGap(in.pair,geometry,out.projection[i],i);
    geometry.plane_distance[i]=v::Dot(geometry.from_secondary,geometry.normal[i]);
    if(out.projection[i].distance_squared<=work.gap[i]*work.gap[i]) {
      if(geometry.plane_distance[i]<=0&&out.primary.intersection!=1&&keep)
        out.primary.cylindrical_gap[i]=1;
      if(geometry.plane_distance[i]>=0&&out.opposite.intersection!=1&&keep)
        out.opposite.cylindrical_gap[i]=1;
    }
  }
  if(!out.active)return;
  const int closest=geometry.frame.triangle?1:SelectClosest(geometry,view,0);
  if(closest>0) {
    const unsigned i=unsigned(closest-1);
    if(out.primary.intersection||out.primary.cylindrical_gap[i])out.primary.subtriangle=closest;
    if(work.opposite_local!=0&&(out.opposite.intersection||out.opposite.cylindrical_gap[i]))
      out.opposite.subtriangle=closest;
  }
}
TL_MATH_HOST_DEVICE inline void ImpactPenetrations(const NativeNewImpactInput& in,
    const ImpactWork& work,NativeNewImpactResult& out) {
  const auto& geometry=work.geometry;
  if(out.primary.subtriangle>0) {
    const unsigned i=unsigned(out.primary.subtriangle-1);
    const double gap=ImpactGap(in.pair,geometry,out.projection[i],i,true);
    const double bb=geometry.plane_distance[i];
    double p=bb>0?g::Max(0.,gap+bb):g::Max(0.,gap-::sqrt(out.projection[i].distance_squared));
    if(out.recontact_intersection>0) {
      const auto arm=geometry.frame.arm[i];
      // Literal donor expression: its x product uses arm(i)*arm(1).
      // Replacing it with a squared Euclidean norm changes the native cutoff.
      const double scale=g::Max(native_constant::em30,
          arm.x*geometry.frame.arm[0].x+arm.y*arm.y+arm.z*arm.z);
      double tolerance=(g::em03*g::em03)*scale;
      if(gap>0)tolerance=g::Min(tolerance,gap*gap);
      if(p*p>tolerance)p=0;
    }
    out.primary.penetration[i]=p;
  }
  if(out.opposite.subtriangle>0) {
    const unsigned i=unsigned(out.opposite.subtriangle-1);
    const double gap=ImpactGap(in.pair,geometry,out.projection[i],i);
    const double bb=geometry.plane_distance[i];
    out.opposite.penetration[i]=bb<0?g::Max(0.,gap-bb):
        g::Max(0.,gap-::sqrt(out.projection[i].distance_squared));
  }
}
} // namespace tlfea::contact::radioss_type25::selection::detail
