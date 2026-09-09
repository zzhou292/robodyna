#pragma once

// Implementation of SurfaceMaterialMeasure's pure density operations. These
// are narrowly scoped cross-product enclosures, not another vector library.
#include "SurfaceMaterialMeasure.h"

namespace tlfea::contact::material_detail {
using Interval=Q4IntegralInterval;

TL_SURFACE_HD inline bool Product(Interval a,Interval b,Interval* output) {
  if (!q4_bounds::Finite(a) || !q4_bounds::Finite(b)) return false;
  Interval next{DBL_MAX,-DBL_MAX};
  const double av[2]={a.lower,a.upper},bv[2]={b.lower,b.upper};
  for (double x:av) for (double y:bv) {
    double lo=0,hi=0;
    if (!q4_bounds::MultiplyScalar(x,y,false,&lo) || !q4_bounds::MultiplyScalar(x,y,true,&hi)) return false;
    if (lo < next.lower) next.lower=lo;
    if (hi > next.upper) next.upper=hi;
  }
  *output=next; return true;
}
TL_SURFACE_HD inline bool SignedScale(Interval a,double b,Interval* output) {
  return Product(a,{b,b},output);
}
TL_SURFACE_HD inline bool Subtract(Interval a,Interval b,Interval* output) {
  return q4_bounds::Add(a,{-b.upper,-b.lower},output);
}
TL_SURFACE_HD inline bool Edge(Vec3 a,Vec3 b,double scale,CrossBounds* output) {
  const double av[3]={a.x,a.y,a.z},bv[3]={b.x,b.y,b.z};
  CrossBounds next;
  for (unsigned c=0;c<3;++c)
    if (!q4_bounds::Difference(av[c],bv[c],&next.component[c]) ||
        !q4_bounds::Scale(next.component[c],scale,&next.component[c])) return false;
  *output=next; return true;
}
TL_SURFACE_HD inline bool Cross(const CrossBounds& a,const CrossBounds& b,CrossBounds* output) {
  CrossBounds next;
  for (unsigned c=0;c<3;++c) {
    const auto j=(c+1)%3,k=(c+2)%3;
    Interval first,second;
    if (!Product(a.component[j],b.component[k],&first) || !Product(a.component[k],b.component[j],&second) ||
        !Subtract(first,second,&next.component[c])) return false;
  }
  *output=next; return true;
}
TL_SURFACE_HD inline bool Norm(const CrossBounds& a,Interval* output) {
  Interval square;
  for (const auto& c:a.component) {
    if (!q4_bounds::Finite(c)) return false;
    const double abslo=::fabs(c.lower),abshi=::fabs(c.upper);
    const double lo=c.lower <= 0 && c.upper >= 0 ? 0 : (abslo < abshi ? abslo : abshi);
    const double hi=abslo > abshi ? abslo : abshi;
    Interval term;
    if (!q4_bounds::MultiplyScalar(lo,lo,false,&term.lower) ||
        !q4_bounds::MultiplyScalar(hi,hi,true,&term.upper) || !q4_bounds::Add(square,term,&square)) return false;
  }
  // Correctly rounded sqrt followed by one outward step; zero stays exact.
  Interval next;
  if (!q4_bounds::Round(::sqrt(square.lower),false,&next.lower) ||
      !q4_bounds::Round(::sqrt(square.upper),true,&next.upper)) return false;
  if (next.lower < 0) next.lower=0;
  *output=next; return true;
}
TL_SURFACE_HD inline bool Dot(const CrossBounds& a,Vec3 direction,Interval* output) {
  const double d[3]={direction.x,direction.y,direction.z};
  Interval sum;
  for (unsigned c=0;c<3;++c) {
    Interval term;
    if (!SignedScale(a.component[c],d[c],&term) || !q4_bounds::Add(sum,term,&sum)) return false;
  }
  *output=sum; return true;
}
TL_SURFACE_HD inline bool Shape(double u,double v,Interval output[4]) {
  const double us[4]={u,-u,-u,u},vs[4]={v,v,-v,-v};
  for (unsigned n=0;n<4;++n) {
    Interval a,b;
    if (!q4_bounds::Difference(1,-us[n],&a) || !q4_bounds::Difference(1,-vs[n],&b) ||
        !q4_bounds::MultiplyPositive(a,b,&output[n]) || !q4_bounds::Scale(output[n],.25,&output[n])) return false;
  }
  return true;
}
TL_SURFACE_HD inline bool At(const Q4MaterialMeasure& reference,double u,double v,CrossBounds* output) {
  Interval shape[4];
  if (!Shape(u,v,shape)) return false;
  CrossBounds next;
  for (unsigned n=0;n<4;++n) for (unsigned c=0;c<3;++c) {
    Interval term;
    if (!Product(reference.corner(n).component[c],shape[n],&term) ||
        !q4_bounds::Add(next.component[c],term,&next.component[c])) return false;
  }
  *output=next; return true;
}
TL_SURFACE_HD inline bool ValidBox(Q4NaturalBox box) {
  return IsFinite(box.u0) && IsFinite(box.u1) && IsFinite(box.v0) && IsFinite(box.v1) &&
         box.u0 >= -1 && box.u1 <= 1 && box.v0 >= -1 && box.v1 <= 1 &&
         box.u0 <= box.u1 && box.v0 <= box.v1;
}
} // namespace tlfea::contact::material_detail

namespace tlfea::contact {
TL_SURFACE_HD inline SurfaceMeasureStatus BoundQ4MaterialDensity(
    const Q4MaterialMeasure& reference,Q4NaturalBox box,Q4IntegralInterval* output) {
  if (!output) return SurfaceMeasureStatus::InvalidOutput;
  if (!reference.prepared()) return SurfaceMeasureStatus::NotPrepared;
  if (!material_detail::ValidBox(box)) return SurfaceMeasureStatus::InvalidInput;
  material_detail::CrossBounds hull;
  for (auto& c:hull.component) c={DBL_MAX,-DBL_MAX};
  double dot_lower=DBL_MAX;
  const double u[2]={box.u0,box.u1},v[2]={box.v0,box.v1};
  for (double x:u) for (double y:v) {
    material_detail::CrossBounds corner;
    Q4IntegralInterval dot;
    if (!material_detail::At(reference,x,y,&corner) ||
        !material_detail::Dot(corner,reference.direction(),&dot)) return SurfaceMeasureStatus::Unrepresentable;
    if (dot.lower < dot_lower) dot_lower=dot.lower;
    for (unsigned c=0;c<3;++c) {
      if (corner.component[c].lower < hull.component[c].lower) hull.component[c].lower=corner.component[c].lower;
      if (corner.component[c].upper > hull.component[c].upper) hull.component[c].upper=corner.component[c].upper;
    }
  }
  // G is affine. Its component hull and fixed-direction dot lower bound hold
  // everywhere in this box; a minimum of corner norms alone would not suffice.
  Q4IntegralInterval norm,projection;
  if (!material_detail::Norm(hull,&norm) || dot_lower <= 0 ||
      !q4_bounds::DividePositive({dot_lower,dot_lower},reference.direction_norm_upper(),&projection))
    return SurfaceMeasureStatus::Unrepresentable;
  if (projection.lower > norm.lower) norm.lower=projection.lower;
  if (!q4_bounds::Nonnegative(norm) || norm.lower <= 0) return SurfaceMeasureStatus::Unrepresentable;
  *output=norm; return SurfaceMeasureStatus::Ok;
}

TL_SURFACE_HD inline SurfaceMeasureStatus EvaluateQ4MaterialDensity(
    const Q4MaterialMeasure& reference,double u,double v,Q4CertifiedIntegral* output) {
  if (!output) return SurfaceMeasureStatus::InvalidOutput;
  Q4IntegralInterval truth;
  const auto status=BoundQ4MaterialDensity(reference,{u,u,v,v},&truth);
  if (status != SurfaceMeasureStatus::Ok) return status;
  double shape[4];
  if (EvaluateQ4Shape(u,v,shape) != Status::kOk) return SurfaceMeasureStatus::InvalidInput;
  Vec3 cross;
  for (unsigned n=0;n<4;++n) cross=Add(cross,Scale(reference.nominal_corner(n),shape[n]));
  const double value=::sqrt(Dot(cross,cross));
  Q4CertifiedIntegral next;
  if (!q4_bounds::Certify(value,truth,&next)) return SurfaceMeasureStatus::Unrepresentable;
  *output=next; return SurfaceMeasureStatus::Ok;
}
} // namespace tlfea::contact
