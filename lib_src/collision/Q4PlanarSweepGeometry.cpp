#include "Q4PlanarSweepGeometry.h"

#include "Q4ContactBounds.h"

#include <algorithm>
#include <initializer_list>

namespace tlfea::contact {
namespace {
using Interval=Q4IntegralInterval;
using PStatus=PlanarContactStatus;

PlanarContactReport Report(PStatus status,const char* message,std::uint32_t parent=UINT32_MAX) {
  return {status,message,parent};
}
bool SameParent(const SurfaceQ4& a,const SurfaceQ4& b) {
  if (a.feature_id != b.feature_id || a.parent_element_id != b.parent_element_id ||
      a.parent_face_id != b.parent_face_id || a.half_thickness != b.half_thickness) return false;
  for (unsigned n=0;n<4;++n) if (a.nodes[n] != b.nodes[n]) return false;
  return true;
}
bool ValidView(const Q4SurfaceView& view,Q4PlanarReferenceView reference) {
  return view.parents && view.parent_count == reference.parent_count && view.positions.valid() &&
         view.velocities.valid() && view.positions.node_count == reference.global_node_count &&
         view.velocities.node_count == reference.global_node_count;
}
bool ValidReference(const PreparedQ4PlanarParent& reference,Q4PlanarReferenceView owner) {
  if (!reference.covered || q4_detail::ValidateParent(reference.parent,owner.global_node_count) != Status::kOk)
    return false;
  const auto* p=reference.reference_projection;
  for (unsigned n=0;n<4;++n) if (!IsFinite(p[n]) || p[n].x != owner.wall_x) return false;
  if (p[0].y != p[3].y || p[1].y != p[2].y || p[0].z != p[1].z || p[2].z != p[3].z ||
      !(p[0].y > p[1].y) || !(p[0].z > p[3].z)) return false;
  Interval width,height,area;
  if (!q4_bounds::Difference(p[0].y,p[1].y,&width) || !q4_bounds::Difference(p[0].z,p[3].z,&height) ||
      !q4_bounds::MultiplyPositive(width,height,&area) || area.lower <= 0) return false;
  // Check the copied producer contract; neither current area nor the swept box
  // is a substitute for this exact same rectangular reference measure.
  const double measure=(p[0].y-p[1].y)*(p[0].z-p[3].z);
  return IsFinite(measure) && measure > 0 && measure == reference.projected_area &&
         reference.area_enclosure.lower == area.lower && reference.area_enclosure.upper == area.upper;
}

// The only signed product needed here is the 2D determinant. Reuse the owning
// contact directed scalar operations, without importing stiffness/mass policy.
bool Product(Interval a,Interval b,Interval* output) {
  if (!q4_bounds::Finite(a) || !q4_bounds::Finite(b)) return false;
  const double x[2]={a.lower,a.upper},y[2]={b.lower,b.upper};
  Interval result;
  for (unsigned i=0;i<2;++i) for (unsigned j=0;j<2;++j) {
    double lower=0,upper=0;
    if (!q4_bounds::MultiplyScalar(x[i],y[j],false,&lower) ||
        !q4_bounds::MultiplyScalar(x[i],y[j],true,&upper)) return false;
    if (i == 0 && j == 0) result={lower,upper};
    else { result.lower=std::min(result.lower,lower); result.upper=std::max(result.upper,upper); }
  }
  *output=result; return true;
}
struct ProjectedEdge { Interval y,z; };
bool HalfEdge(Vec3 positive,Vec3 negative,ProjectedEdge* output) {
  ProjectedEdge edge;
  if (!q4_bounds::Difference(positive.y,negative.y,&edge.y) || !q4_bounds::Scale(edge.y,.5,&edge.y) ||
      !q4_bounds::Difference(positive.z,negative.z,&edge.z) || !q4_bounds::Scale(edge.z,.5,&edge.z)) return false;
  *output=edge; return true;
}
bool Determinant(const ProjectedEdge& du,const ProjectedEdge& dv,Interval* output) {
  Interval positive,negative;
  return Product(du.y,dv.z,&positive) && Product(du.z,dv.y,&negative) &&
         q4_bounds::Add(positive,{-negative.upper,-negative.lower},output);
}
bool JacobianHull(const Vec3 base[4],const Vec3 candidate[4],Interval* output) {
  ProjectedEdge du[2][2],dv[2][2];
  const Vec3* endpoint[2]={base,candidate};
  for (unsigned t=0;t<2;++t) {
    // At natural edges v=+1/-1 and u=+1/-1, derivatives are half-edge
    // differences. No rounded global-coordinate weighted sum is required.
    const auto* p=endpoint[t];
    if (!HalfEdge(p[0],p[1],&du[t][0]) || !HalfEdge(p[3],p[2],&du[t][1]) ||
        !HalfEdge(p[0],p[3],&dv[t][0]) || !HalfEdge(p[1],p[2],&dv[t][1])) return false;
  }
  Interval hull;
  for (unsigned u=0;u<2;++u) for (unsigned v=0;v<2;++v) {
    Interval coefficient[3],cross01,cross10;
    if (!Determinant(du[0][v],dv[0][u],&coefficient[0]) ||
        !Determinant(du[1][v],dv[1][u],&coefficient[2]) ||
        !Determinant(du[0][v],dv[1][u],&cross01) ||
        !Determinant(du[1][v],dv[0][u],&cross10) ||
        !q4_bounds::Add(cross01,cross10,&coefficient[1]) ||
        !q4_bounds::Scale(coefficient[1],.5,&coefficient[1])) return false;
    // J(u,v,t) is affine in u,v (the uv term cancels algebraically), and
    // quadratic in t. Bernstein weights are (1-t)^2,2t(1-t),t^2, all positive
    // and summing to one. The four parameter-corner coefficient hulls therefore
    // enclose J over the complete parent and complete prescribed linear step.
    if (u == 0 && v == 0) hull=coefficient[0];
    for (const auto c:coefficient) {
      hull.lower=std::min(hull.lower,c.lower); hull.upper=std::max(hull.upper,c.upper);
    }
  }
  *output=hull; return true;
}
PlanarContactReport CheckJacobian(const Vec3 base[4],const Vec3 candidate[4],double minimum_ratio,
                                 std::uint32_t index,Q4PlanarParentSweep* result) {
  Interval reference_jacobian;
  if (!JacobianHull(base,candidate,&result->projected_jacobian) ||
      !q4_bounds::Scale(result->reference_material_area_enclosure,.25,&reference_jacobian) ||
      reference_jacobian.lower <= 0 ||
      !q4_bounds::MultiplyScalar(minimum_ratio,reference_jacobian.upper,true,&result->required_jacobian_lower) ||
      result->required_jacobian_lower <= 0)
    return Report(PStatus::InvalidInput,"Projected Jacobian enclosure is unrepresentable",index);
  if (result->projected_jacobian.lower < result->required_jacobian_lower)
    return Report(PStatus::UnsupportedMotion,"Unresolved Bernstein lower bound for the declared projected Jacobian ratio",index);
  Interval lower_ratio,upper_ratio;
  if (!q4_bounds::DividePositive(result->projected_jacobian,reference_jacobian.upper,&lower_ratio) ||
      !q4_bounds::DividePositive(result->projected_jacobian,reference_jacobian.lower,&upper_ratio))
    return Report(PStatus::InvalidInput,"Projected Jacobian ratio enclosure is unrepresentable",index);
  result->projected_jacobian_ratio={lower_ratio.lower,upper_ratio.upper};
  return Report(PStatus::Ok,"Projected Jacobian sweep certified",index);
}
PlanarContactReport CheckCoverage(const PlanarWallGeometry& wall,double clearance,std::uint32_t index,
                                  const Q4PlanarParentSweep& result) {
  const auto lo=result.projected_minimum,hi=result.projected_maximum;
  const Vec3 corner[4]={{lo.x,hi.y,hi.z},{lo.x,lo.y,hi.z},{lo.x,lo.y,lo.z},{lo.x,hi.y,lo.z}};
  constexpr unsigned triangles[2][3]={{0,1,2},{0,2,3}};
  for (const auto& nodes:triangles) {
    TriangleGeometry triangle; triangle.face_id=result.parent.feature_id;
    for (unsigned n=0;n<3;++n) {
      triangle.vertices[n]=corner[nodes[n]];
      triangle.vertex_ids[n]=nodes[n];  // Local box geometry keys, never source node identities.
    }
    bool covered=false;
    auto report=wall.ClassifyTriangle(triangle,clearance,&covered);
    if (report.status != PStatus::Ok) { report.sample=index; return report; }
    if (!covered) return Report(PStatus::UnsupportedGeometry,"Swept projected box is outside the finite wall",index);
  }
  return Report(PStatus::Ok,"Swept projected box is wholly covered",index);
}
}  // namespace

PlanarContactReport CheckQ4PlanarSweep(
    const PlanarWallGeometry& wall,Q4PlanarReferenceView reference,
    const Q4SurfaceView& base,const Q4SurfaceView& candidate,
    const Q4PlanarSweepLimits& limits,Q4PlanarSweepGeometry* output) {
  if (!output) return Report(PStatus::InvalidOutput,"Missing Q4 sweep output");
  if (!wall.initialized()) return Report(PStatus::NotInitialized,"Finite wall has not been prepared");
  if (!reference.parents || !reference.parent_count || reference.parent_count > MaxQ4PlanarParents ||
      !reference.global_node_count || reference.wall_x != wall.wall_x() || reference.wall_tolerance != wall.tolerance())
    return Report(PStatus::NotInitialized,"Invalid or mismatched prepared Q4 reference");
  if (!ValidView(base,reference) || !ValidView(candidate,reference) || !IsFinite(limits.exposed_clearance) ||
      !(limits.exposed_clearance > 8*wall.tolerance()) || !IsFinite(limits.minimum_projected_jacobian_ratio) ||
      limits.minimum_projected_jacobian_ratio <= 0 || limits.minimum_projected_jacobian_ratio > 1)
    return Report(PStatus::InvalidInput,"Invalid endpoint views or explicit Q4 sweep limits");
  Q4PlanarSweepGeometry staged;
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto& saved=reference.parents[p];
    if (!ValidReference(saved,reference))
      return Report(PStatus::UnsupportedGeometry,"Invalid covered rectangular reference or material measure",p);
    for (std::uint32_t previous=0;previous<p;++previous)
      if (saved.parent.feature_id == reference.parents[previous].parent.feature_id ||
          saved.parent.parent_element_id == reference.parents[previous].parent.parent_element_id)
        return Report(PStatus::InvalidInput,"Duplicate reference Q4 feature or parent identity",p);
    if (!SameParent(base.parents[p],saved.parent) || !SameParent(candidate.parents[p],saved.parent))
      return Report(PStatus::UnsupportedGeometry,"Changed Q4 source identity, connectivity or zero offset",p);
    auto& result=staged.parents[p]; result.parent=saved.parent;
    result.reference_material_area=saved.projected_area;
    result.reference_material_area_enclosure=saved.area_enclosure;
    Vec3 x0[4],x1[4];
    for (unsigned n=0;n<4;++n) {
      const auto node=saved.parent.nodes[n];
      x0[n]=base.positions.at(node); x1[n]=candidate.positions.at(node);
      if (!IsFinite(x0[n]) || !IsFinite(x1[n]) ||
          !IsFinite(base.velocities.at(node)) || !IsFinite(candidate.velocities.at(node)))
        return Report(PStatus::InvalidInput,"Nonfinite Q4 endpoint position or velocity",p);
      if (n == 0) result.projected_minimum=result.projected_maximum={wall.wall_x(),x0[n].y,x0[n].z};
      for (const auto x:{x0[n],x1[n]}) {
        result.projected_minimum.y=std::min(result.projected_minimum.y,x.y);
        result.projected_minimum.z=std::min(result.projected_minimum.z,x.z);
        result.projected_maximum.y=std::max(result.projected_maximum.y,x.y);
        result.projected_maximum.z=std::max(result.projected_maximum.z,x.z);
      }
    }
    const auto jacobian=CheckJacobian(x0,x1,limits.minimum_projected_jacobian_ratio,p,&result);
    if (jacobian.status != PStatus::Ok) return jacobian;
    const auto coverage=CheckCoverage(wall,limits.exposed_clearance,p,result);
    if (coverage.status != PStatus::Ok) return coverage;
  }
  staged.parent_count=reference.parent_count; staged.valid=true; *output=staged;
  return Report(PStatus::Ok,"Q4 swept finite-wall coverage and positive projection certified");
}
}  // namespace tlfea::contact
