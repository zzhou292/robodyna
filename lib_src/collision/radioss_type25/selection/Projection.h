// SPDX-License-Identifier: AGPL-3.0-or-later
// Native I25DST3_1 projection algebra, including unused-but-defined T3 raw sectors.
#pragma once
#include "Prepare.h"
namespace tlfea::contact::radioss_type25::selection::detail {
template<class Result>
TL_MATH_HOST_DEVICE inline void RawProjection(Work& w, Result& out) {
  Vector crossed[4];
  for(unsigned i=0;i<4;++i)crossed[i]=v::Cross(w.from_secondary,w.relative[i]);
  for(unsigned i=0;i<4;++i) {
    const unsigned next=(i+1)%4;
    const auto normal=v::Cross(w.frame.arm[i],w.frame.arm[next]);
    const double inverse=1./g::Max(native_constant::em30,::sqrt(v::Dot(normal,normal)));
    w.normal[i]=v::Scale(normal,inverse);
    out.sector[i].raw_lb=-v::Dot(w.normal[i],crossed[next])*inverse;
    out.sector[i].raw_lc=v::Dot(w.normal[i],crossed[i])*inverse;
    out.sector[i].defined|=RawBarycentricDefined;
  }
  for(unsigned i=0;i<4;++i) {
    const unsigned previous=(i+3)%4;
    const double inverse=1./g::Max(native_constant::em30,v::Dot(w.frame.arm[i],w.frame.arm[i]));
    const double lc=out.sector[i].raw_lc,lb=out.sector[previous].raw_lb;
    w.hlc[i]=lc*::fabs(lc)*inverse;w.hlb[previous]=lb*::fabs(lb)*inverse;
    w.along[i]=g::Max(0.,g::Min(1.,-v::Dot(w.from_secondary,w.frame.arm[i])*inverse));
  }
}
template<class Result>
TL_MATH_HOST_DEVICE inline double ProjectSector(const NativePairInput& in,Work& w,
    Result& out,unsigned i,bool radiation_first=false) {
  const unsigned j=(i+1)%4;
  auto& s=out.sector[i];
  const auto edge=v::Subtract(w.frame.point[j],w.frame.point[i]);
  s.clamped_lb=s.raw_lb;s.clamped_lc=s.raw_lc;
  double la=1.-s.clamped_lb-s.clamped_lc;
  const double inverse=1./g::Max(g::em20,v::Dot(edge,edge));
  const double hla=la*::fabs(la)*inverse;
  if(la<0&&hla<=w.hlb[i]&&hla<=w.hlc[i]) {
    s.clamped_lb=g::Max(0.,g::Min(1.,v::Dot(w.relative[j],edge)*inverse));
    s.clamped_lc=1.-s.clamped_lb;
  } else if(s.clamped_lb<0&&w.hlb[i]<=w.hlc[i]&&w.hlb[i]<=hla) {
    s.clamped_lb=0;s.clamped_lc=w.along[j];
  } else if(s.clamped_lc<0&&w.hlc[i]<=hla&&w.hlc[i]<=w.hlb[i]) {
    s.clamped_lc=0;s.clamped_lb=w.along[i];
  }
  s.defined|=ClampedBarycentricDefined;
  la=1.-s.clamped_lb-s.clamped_lc;
  const auto point=v::Add(v::Add(v::Scale(w.frame.point[4],la),
      v::Scale(w.frame.point[i],s.clamped_lb)),v::Scale(w.frame.point[j],s.clamped_lc));
  const auto delta=v::Subtract(in.secondary,point);
  s.distance_squared=v::Dot(delta,delta);
  const double uncapped=in.secondary_gap+la*w.center_gap+s.clamped_lb*in.main_gap[i]+
      s.clamped_lc*in.main_gap[j]+in.applied_gap;
  // Source reverses MAX's argument order for Q4 sectors2..4; keep it.
  const double inner=(radiation_first||i==0)?g::Max(in.radiation_range,uncapped):g::Max(uncapped,in.radiation_range);
  const double gap=g::Min(inner,g::Max(in.radiation_range,
      native_constant::ep20*native_constant::ep10+in.applied_gap));
  w.plane_distance[i]=v::Dot(w.from_secondary,w.normal[i]);
  s.penetration=w.plane_distance[i]>0 ? g::Max(0.,gap+w.plane_distance[i]) :
      g::Max(0.,gap-::sqrt(s.distance_squared));
  return gap;
}
} // namespace tlfea::contact::radioss_type25::selection::detail
