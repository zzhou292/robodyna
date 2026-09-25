// SPDX-License-Identifier: AGPL-3.0-or-later
// Reimplementation of OpenRadioss I25PEN3, a62b27e6, MYREAL8.
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::candidates {
namespace detail {
namespace v = tl::math::fixed3;
inline constexpr double em20 = 1. / native_constant::ep20;
inline constexpr double em03 = 1. / 1000.;
TL_MATH_HOST_DEVICE inline double Max(double a,double b) {return a<b?b:a;}
TL_MATH_HOST_DEVICE inline double Min(double a,double b) {return a<b?a:b;}
TL_MATH_HOST_DEVICE inline bool Nonnegative(double a) {return tl::math::Finite(a)&&a>=0;}
TL_MATH_HOST_DEVICE inline bool SameBits(double a,double b) {
  const auto* first=reinterpret_cast<const unsigned char*>(&a);
  const auto* second=reinterpret_cast<const unsigned char*>(&b);
  for(unsigned i=0;i<sizeof(double);++i)if(first[i]!=second[i])return false;
  return true;
}
TL_MATH_HOST_DEVICE inline bool SameVector(Vector a,Vector b) {
  return SameBits(a.x,b.x)&&SameBits(a.y,b.y)&&SameBits(a.z,b.z);
}
TL_MATH_HOST_DEVICE inline double Clamp(double a) {return Max(0.,Min(1.,a));}
TL_MATH_HOST_DEVICE inline Bounds Box(const Vector* p) {
  Bounds b{p[0],p[0]};
  for(unsigned i=1;i<4;++i) {
    b.minimum={Min(b.minimum.x,p[i].x),Min(b.minimum.y,p[i].y),Min(b.minimum.z,p[i].z)};
    b.maximum={Max(b.maximum.x,p[i].x),Max(b.maximum.y,p[i].y),Max(b.maximum.z,p[i].z)};
  }
  return b;
}
// One native fan sector, preserving the source's two distinct denominator floors
// and its ordered mutually exclusive edge-region decisions.
TL_MATH_HOST_DEVICE inline bool Sector(Vector center,Vector a,Vector b,Vector point,
                                       double gap2,double& output) {
  const auto a0=v::Subtract(a,center), b0=v::Subtract(b,center);
  const auto i0=v::Subtract(center,point), ia=v::Subtract(a,point), ib=v::Subtract(b,point);
  const auto sa=v::Cross(i0,ia),sb=v::Cross(i0,ib),s0=v::Cross(a0,b0);
  const double norm=v::Dot(s0,s0),a2=v::Dot(a0,a0),b2=v::Dot(b0,b0);
  if(!v::Finite(a0)||!v::Finite(b0)||!v::Finite(i0)||!v::Finite(ia)||!v::Finite(ib)||
     !v::Finite(sa)||!v::Finite(sb)||!v::Finite(s0)||!Nonnegative(norm)||
     !Nonnegative(a2)||!Nonnegative(b2))return false;
  const double s2=1./Max(native_constant::em30,norm);
  double lb=-v::Dot(s0,sb)*s2,lc=v::Dot(s0,sa)*s2;
  double inverse=1./Max(native_constant::em30,a2);
  const double hlc=lc*::fabs(lc)*inverse;
  const double alpha_a_unclamped=-v::Dot(i0,a0)*inverse;
  const double alpha_a=Clamp(alpha_a_unclamped);
  inverse=1./Max(native_constant::em30,b2);
  const double hlb=lb*::fabs(lb)*inverse;
  const double alpha_b_unclamped=-v::Dot(i0,b0)*inverse;
  const double alpha_b=Clamp(alpha_b_unclamped);
  const auto ab=v::Subtract(b,a);
  const double la=1.-lb-lc,ab2=v::Dot(ab,ab);
  inverse=1./Max(em20,ab2);
  const double hla=la*::fabs(la)*inverse;
  if(!tl::math::Finite(lb)||!tl::math::Finite(lc)||!tl::math::Finite(hlc)||
     !tl::math::Finite(hlb)||!tl::math::Finite(alpha_a_unclamped)||
     !tl::math::Finite(alpha_b_unclamped)||!v::Finite(ab)||!Nonnegative(ab2)||
     !tl::math::Finite(la)||!tl::math::Finite(hla))return false;
  if(la<0.&&hla<=hlb&&hla<=hlc) {
    lb=v::Dot(ib,ab)*inverse;
    if(!tl::math::Finite(lb))return false;
    lb=Clamp(lb);lc=1.-lb;
  } else if(lb<0.&&hlb<=hlc&&hlb<=hla) {lb=0.;lc=alpha_b;}
  else if(lc<0.&&hlc<=hla&&hlc<=hlb) {lc=0.;lb=alpha_a;}
  const auto normal=v::Subtract(point,v::Add(v::Add(center,v::Scale(a0,lb)),v::Scale(b0,lc)));
  const double distance2=v::Dot(normal,normal),clearance=gap2-distance2;
  if(!v::Finite(normal)||!Nonnegative(distance2)||!tl::math::Finite(clearance))return false;
  output=Max(0.,clearance);return true;
}
} // namespace detail
TL_MATH_HOST_DEVICE inline Status EvaluatePacked(const PackedRow& row,FilterResult* output) {
  namespace d=detail;namespace v=tl::math::fixed3;
  if(!output||!d::Nonnegative(row.gap)||!d::Nonnegative(row.margin)||
     !v::Finite(row.secondary)||row.main_count<=0||row.symmetry<0||row.symmetry>7)
    return Status::InvalidInput;
  for(unsigned i=0;i<4;++i) {
    if(!row.nodes[i]||!v::Finite(row.vertices[i]))return Status::InvalidInput;
    for(unsigned j=0;j<i;++j)if(row.nodes[i]==row.nodes[j]&&
       !d::SameVector(row.vertices[i],row.vertices[j]))return Status::InvalidInput;
  }
  const double zone=row.gap+row.margin,gap2=zone*zone;
  if(!d::Nonnegative(zone)||!d::Nonnegative(gap2))return Status::NonfiniteResult;
  const bool triangle=row.nodes[2]==row.nodes[3];
  Vector center=triangle?row.vertices[2]:v::Scale(v::Add(v::Add(v::Add(
      row.vertices[0],row.vertices[1]),row.vertices[2]),row.vertices[3]),.25);
  if(!v::Finite(center))return Status::NonfiniteResult;
  FilterResult next;
  for(unsigned sector=0;sector<(triangle?1u:4u);++sector) {
    double clearance=0;
    if(!d::Sector(center,row.vertices[sector],row.vertices[(sector+1)%4],
                  row.secondary,gap2,clearance))return Status::NonfiniteResult;
    next.squared_clearance=d::Max(next.squared_clearance,clearance);
  }
  if(row.segment_type==0||row.segment_type>row.main_count) {
    const auto box=d::Box(row.vertices);const auto size=v::Subtract(box.maximum,box.minimum);
    const double dd=d::Max(d::Max(size.x,size.y),size.z);
    if(!v::Finite(size)||!d::Nonnegative(dd))return Status::NonfiniteResult;
    Vector max_delta{};
    for(unsigned i=0;i<4;++i) {
      const auto delta=v::Subtract(row.secondary,row.vertices[i]);
      if(!v::Finite(delta))return Status::NonfiniteResult;
      max_delta={d::Max(max_delta.x,::fabs(delta.x)),d::Max(max_delta.y,::fabs(delta.y)),
                 d::Max(max_delta.z,::fabs(delta.z))};
    }
    // Native ICOD bit1 is Z, bit2 Y, bit4 X.
    if(((row.symmetry&1)&&max_delta.z<d::em03*dd)||
       ((row.symmetry&2)&&max_delta.y<d::em03*dd)||
       ((row.symmetry&4)&&max_delta.x<d::em03*dd))next.squared_clearance=0.;
  }
  next.included=next.squared_clearance!=0.;*output=next;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::candidates
