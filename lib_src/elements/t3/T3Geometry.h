// SPDX-License-Identifier: AGPL-3.0-or-later
// C3EVEC3 selected arithmetic, OpenRadioss (C) 2026 Siemens; see LICENSE.md.
#pragma once
#include "T3Data.h"
#include "lib_src/math/Quaternion.h" // Shared binary64 finite predicate.
#include <cfloat>
#include <cmath>

namespace tl::fea::t3::detail {
constexpr double MaximumCoordinate=1e6,MinimumEdge=1e-9,MaximumEdge=1e3;
constexpr double MinimumEdgeRatio=1e-6,MinimumNormalizedTwiceArea=1e-6;
constexpr double AcosMargin=1e-12,GuardBand=64*DBL_EPSILON;
// All powers up to EP15 are exact native integer products in binary64.
constexpr double Em15=1/(100000.*100000.*100000.);
TL_T3_HD inline bool Positive(double x) { return tl::math::Finite(x)&&x>0; }
TL_T3_HD inline bool Finite(Vec3 x) {
  return tl::math::Finite(x.x)&&tl::math::Finite(x.y)&&tl::math::Finite(x.z);
}
TL_T3_HD inline Vec3 Difference(Vec3 a,Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
TL_T3_HD inline double Norm(Vec3 x) { return ::sqrt(x.x*x.x+x.y*x.y+x.z*x.z); }
TL_T3_HD inline double Project(const Matrix3& f,unsigned axis,Vec3 x) {
  return f.v[axis]*x.x+f.v[3+axis]*x.y+f.v[6+axis]*x.z;
}
TL_T3_HD inline bool Coordinates(const Vec3 (&x)[3]) {
  for(auto p:x) if(!Finite(p)||::fabs(p.x)>MaximumCoordinate||
      ::fabs(p.y)>MaximumCoordinate||::fabs(p.z)>MaximumCoordinate) return false;
  return true;
}
// FP64-only numerical admission, deliberately narrower than the native host
// long-double preflight. Unit-scale ratios carry an additive 64epsilon band;
// this is not an exact geometry certificate or wall-projection condition.
TL_T3_HD inline bool SupportedGeometry(const Vec3 (&x)[3],double& longest) {
  Vec3 e[3]; double lo=MaximumEdge,hi=0;
  for(unsigned i=0;i<3;++i) {
    e[i]=Difference(x[(i+1)%3],x[i]); const double length=Norm(e[i]);
    if(!Positive(length)||!(length>MinimumEdge*(1+GuardBand))||
       !(length<MaximumEdge*(1-GuardBand))) return false;
    lo=::fmin(lo,length); hi=::fmax(hi,length);
  }
  if(!(lo/hi>MinimumEdgeRatio+GuardBand)) return false;
  const Vec3 a{e[0].x/hi,e[0].y/hi,e[0].z/hi};
  const Vec3 b{e[1].x/hi,e[1].y/hi,e[1].z/hi};
  const Vec3 cross{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
  if(!(Norm(cross)>MinimumNormalizedTwiceArea+GuardBand)) return false;
  longest=hi; return true;
}
TL_T3_HD inline bool Proper(const Matrix3& f) {
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
    double dot=0; for(unsigned k=0;k<3;++k) dot+=f.v[3*k+i]*f.v[3*k+j];
    if(!tl::math::Finite(dot)||::fabs(dot-(i==j?1.:0.))>5e-13) return false;
  }
  const double det=f.v[0]*(f.v[4]*f.v[8]-f.v[5]*f.v[7])
    -f.v[1]*(f.v[3]*f.v[8]-f.v[5]*f.v[6])+f.v[2]*(f.v[3]*f.v[7]-f.v[4]*f.v[6]);
  return tl::math::Finite(det)&&::fabs(det-1)<=5e-13;
}
struct FrameWork { Matrix3 frame; Vec3 edge21,edge31; double area=0,length21=0; };
// Same selected starter and engine C3EVEC3 arithmetic: IREP0/IDRAPE0,
// ISH3NFRAM1. Native e2 is normalized independently; do not replace this with
// a quaternion frame or QEPH CLSKEW3. Caller has checked the numerical domain.
TL_T3_HD inline Status NativeFrame(const Vec3 (&x)[3],FrameWork& out) {
  FrameWork w; w.edge21=Difference(x[1],x[0]); w.edge31=Difference(x[2],x[0]);
  const auto edge32=Difference(x[2],x[1]);
  auto e1=w.edge21; w.length21=Norm(e1);
  e1.x=e1.x/w.length21; e1.y=e1.y/w.length21; e1.z=e1.z/w.length21;
  Vec3 e3{w.edge31.y*edge32.z-w.edge31.z*edge32.y,
    w.edge31.z*edge32.x-w.edge31.x*edge32.z,
    w.edge31.x*edge32.y-w.edge31.y*edge32.x};
  double sum=Norm(e3);
  e3.x=e3.x/sum; e3.y=e3.y/sum; e3.z=e3.z/sum; w.area=.5*sum;
  Vec3 e2{e3.y*e1.z-e3.z*e1.y,e3.z*e1.x-e3.x*e1.z,e3.x*e1.y-e3.y*e1.x};
  sum=Norm(e2); e2.x=e2.x/sum; e2.y=e2.y/sum; e2.z=e2.z/sum;
  w.frame={{e1.x,e2.x,e3.x,e1.y,e2.y,e3.y,e1.z,e2.z,e3.z}};
  if(!Positive(w.area)||!Positive(w.length21)||!Proper(w.frame)) return Status::kNonfiniteResult;
  out=w; return Status::kSuccess;
}
TL_T3_HD inline double CharacteristicLength(double x2,double x3,double y3,double area) {
  const double al1=x2*x2,al2=(x3-x2)*(x3-x2)+y3*y3,al3=x3*x3+y3*y3;
  return 2*area/::sqrt(::fmax(::fmax(al1,al2),al3));
}
} // namespace tl::fea::t3::detail
