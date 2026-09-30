#pragma once

#include "Q4ContactBounds.h"
#include "Q4SurfaceMapping.h"

namespace tlfea::contact {

enum class SurfaceMeasureStatus {
  Ok, InvalidInput, InvalidOutput, NotPrepared, UnresolvedGeometry, Unrepresentable
};
struct Q4NaturalBox { double u0=-1,u1=1,v0=-1,v1=1; };

namespace material_detail {
// Three scalar enclosures, solely for the intrinsic reference cross product.
struct CrossBounds { Q4IntegralInterval component[3]{}; };
}

// Immutable copied midsurface reference. No wall projection, mass, director,
// dynamics owner or integration history. The natural Q4 order is the existing
// C1 order (+,+),(-,+),(-,-),(+,-); source connectivity is never reordered.
// Host preparation admits a sufficient fixed-direction regularity chart:
// G(u,v)=Xu x Xv is affine, and its four corner dot products with one fixed
// intrinsic direction must have strictly positive directed lower bounds.
// Rejection means unresolved in this chart, not proof of a folded shell.
class Q4MaterialMeasure {
 public:
  TL_SURFACE_HD bool prepared() const { return prepared_; }
  TL_SURFACE_HD const SurfaceQ4& parent() const { return parent_; }
  TL_SURFACE_HD Vec3 position(unsigned n) const { return positions_[n]; }
  TL_SURFACE_HD const material_detail::CrossBounds& corner(unsigned n) const { return corners_[n]; }
  TL_SURFACE_HD Vec3 nominal_corner(unsigned n) const { return nominal_[n]; }
  TL_SURFACE_HD Vec3 direction() const { return direction_; }
  TL_SURFACE_HD double direction_norm_upper() const { return direction_norm_upper_; }
  TL_SURFACE_HD Q4IntegralInterval area_enclosure() const { return area_; }
 private:
  friend SurfaceMeasureStatus PrepareQ4MaterialMeasure(VectorView,const SurfaceQ4&,Q4MaterialMeasure*);
  SurfaceQ4 parent_;
  Vec3 positions_[4]{},nominal_[4]{},direction_;
  material_detail::CrossBounds corners_[4]{};
  double direction_norm_upper_=0;
  Q4IntegralInterval area_;
  bool prepared_=false;
};

// Native linear triangle: natural simplex (r,s)>=0, r+s<=1. Its density is
// |(X1-X0) x (X2-X0)|, and its total reference area is half that density.
// There is no repeated-fourth-node proxy or display-triangle decomposition.
class T3MaterialMeasure {
 public:
  TL_SURFACE_HD bool prepared() const { return prepared_; }
  TL_SURFACE_HD const SurfaceTriangle& parent() const { return parent_; }
  TL_SURFACE_HD Vec3 position(unsigned n) const { return positions_[n]; }
  TL_SURFACE_HD Q4CertifiedIntegral density() const { return density_; }
  TL_SURFACE_HD Q4IntegralInterval area_enclosure() const { return area_; }
 private:
  friend SurfaceMeasureStatus PrepareT3MaterialMeasure(VectorView,const SurfaceTriangle&,T3MaterialMeasure*);
  SurfaceTriangle parent_;
  Vec3 positions_[3]{};
  Q4CertifiedIntegral density_;
  Q4IntegralInterval area_;
  bool prepared_=false;
};

// Host startup operations. Zero offset and finite, distinct indexed geometry
// are required; arbitrary 3D orientation, skew and regular Q4 warping are legal.
// Every failure preserves the complete output. Input allocation/lifetime and
// nonaliasing remain caller contracts. No node buffers are changed.
SurfaceMeasureStatus PrepareQ4MaterialMeasure(VectorView,const SurfaceQ4&,Q4MaterialMeasure*);
SurfaceMeasureStatus PrepareT3MaterialMeasure(VectorView,const SurfaceTriangle&,T3MaterialMeasure*);

// Pure bounded host/device evaluation, no allocation or quadrature. Bounds
// enclose the exact real density of the represented reference coordinates.
// The Q4 whole-area enclosure is only a coarse bound (4 times root-density
// bounds); it must NEVER replace the varying density in a warped integral.
// Natural boxes may collapse to a point, but must lie in [-1,1]^2. Arithmetic
// requirements are those of Q4ContactBounds (binary64 RN, no FTZ/fast-math).
TL_SURFACE_HD inline SurfaceMeasureStatus BoundQ4MaterialDensity(
    const Q4MaterialMeasure&,Q4NaturalBox,Q4IntegralInterval*);
TL_SURFACE_HD inline SurfaceMeasureStatus EvaluateQ4MaterialDensity(
    const Q4MaterialMeasure&,double u,double v,Q4CertifiedIntegral*);

} // namespace tlfea::contact

#include "SurfaceMaterialBounds.h"
