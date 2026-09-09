#include "SurfaceMaterialMeasure.h"
#include "SurfaceContactGeometry.h"

namespace tlfea::contact {
namespace {
using Code=SurfaceMeasureStatus;
using Bounds=material_detail::CrossBounds;
bool FixedDirection(Vec3 center,Vec3* direction,double* upper_norm) {
  if (!IsFinite(center)) return false;
  const double scale=geometry_detail::MaxAbs(center);
  if (!(scale > 0)) return false;
  const Vec3 next{center.x/scale,center.y/scale,center.z/scale};
  if (!IsFinite(next)) return false;
  Bounds exact{{{next.x,next.x},{next.y,next.y},{next.z,next.z}}};
  Q4IntegralInterval norm;
  if (!material_detail::Norm(exact,&norm) || !(norm.lower > 0)) return false;
  *direction=next; *upper_norm=norm.upper; return true;
}
bool NativeTriangle(const SurfaceTriangle& p,std::uint32_t count) {
  if (!p.feature_id || !p.parent_element_id || p.half_thickness != 0 ||
      p.interpolation != SurfaceInterpolation::kLinearTriangle) return false;
  for (unsigned n=0;n<3;++n) {
    if (p.nodes[n] >= count) return false;
    for (unsigned previous=0;previous<n;++previous) if (p.nodes[n] == p.nodes[previous]) return false;
  }
  return true;
}
} // namespace

SurfaceMeasureStatus PrepareQ4MaterialMeasure(VectorView positions,const SurfaceQ4& parent,
                                              Q4MaterialMeasure* output) {
  if (!output) return Code::InvalidOutput;
  if (!positions.valid() || q4_detail::ValidateParent(parent,positions.node_count) != Status::kOk)
    return Code::InvalidInput;
  Q4MaterialMeasure next; next.parent_=parent;
  for (unsigned n=0;n<4;++n) {
    next.positions_[n]=positions.at(parent.nodes[n]);
    if (!IsFinite(next.positions_[n])) return Code::InvalidInput;
  }
  const auto* p=next.positions_;
  // Half-edge derivatives at the natural corners; both signs agree with the
  // existing Q4 mapping. Translation cancels before any product is formed.
  const Vec3 du[2]={Scale(Subtract(p[0],p[1]),.5),Scale(Subtract(p[3],p[2]),.5)};
  const Vec3 dv[2]={Scale(Subtract(p[0],p[3]),.5),Scale(Subtract(p[1],p[2]),.5)};
  Bounds ub[2],vb[2];
  if (!material_detail::Edge(p[0],p[1],.5,&ub[0]) || !material_detail::Edge(p[3],p[2],.5,&ub[1]) ||
      !material_detail::Edge(p[0],p[3],.5,&vb[0]) || !material_detail::Edge(p[1],p[2],.5,&vb[1]))
    return Code::Unrepresentable;
  constexpr unsigned ui[4]={0,0,1,1},vi[4]={0,1,1,0};
  for (unsigned n=0;n<4;++n) {
    if (!material_detail::Cross(ub[ui[n]],vb[vi[n]],&next.corners_[n])) return Code::Unrepresentable;
    next.nominal_[n]=geometry_detail::Cross(du[ui[n]],dv[vi[n]]);
    if (!IsFinite(next.nominal_[n])) return Code::Unrepresentable;
  }
  const Vec3 center=geometry_detail::Cross(Scale(Add(du[0],du[1]),.5),Scale(Add(dv[0],dv[1]),.5));
  if (!FixedDirection(center,&next.direction_,&next.direction_norm_upper_)) return Code::UnresolvedGeometry;
  for (const auto& corner:next.corners_) {
    Q4IntegralInterval dot;
    if (!material_detail::Dot(corner,next.direction_,&dot)) return Code::Unrepresentable;
    if (!(dot.lower > 0)) return Code::UnresolvedGeometry;
  }
  next.prepared_=true;  // Private candidate only; public object remains intact.
  Q4IntegralInterval density;
  const auto status=BoundQ4MaterialDensity(next,{},&density);
  if (status != Code::Ok) return status;
  if (!q4_bounds::Scale(density,4,&next.area_)) return Code::Unrepresentable;
  *output=next; return Code::Ok;
}

SurfaceMeasureStatus PrepareT3MaterialMeasure(VectorView positions,const SurfaceTriangle& parent,
                                              T3MaterialMeasure* output) {
  if (!output) return Code::InvalidOutput;
  if (!positions.valid() || !NativeTriangle(parent,positions.node_count)) return Code::InvalidInput;
  T3MaterialMeasure next; next.parent_=parent;
  for (unsigned n=0;n<3;++n) {
    next.positions_[n]=positions.at(parent.nodes[n]);
    if (!IsFinite(next.positions_[n])) return Code::InvalidInput;
  }
  Bounds a,b,cross;
  Q4IntegralInterval norm;
  if (!material_detail::Edge(next.positions_[1],next.positions_[0],1,&a) ||
      !material_detail::Edge(next.positions_[2],next.positions_[0],1,&b) ||
      !material_detail::Cross(a,b,&cross) || !material_detail::Norm(cross,&norm)) return Code::Unrepresentable;
  if (!(norm.lower > 0)) return Code::UnresolvedGeometry;
  const auto nominal=geometry_detail::Cross(Subtract(next.positions_[1],next.positions_[0]),
                                           Subtract(next.positions_[2],next.positions_[0]));
  if (!q4_bounds::Certify(::sqrt(Dot(nominal,nominal)),norm,&next.density_) ||
      !q4_bounds::Scale(norm,.5,&next.area_)) return Code::Unrepresentable;
  next.prepared_=true; *output=next; return Code::Ok;
}
} // namespace tlfea::contact
