#include "PreparedPlanarWallQuery.h"
#include "Q4ContactBounds.h"
#include <algorithm>

namespace tlfea::contact {
namespace {
// These are acceleration eligibility limits, never new geometry acceptance
// limits. Their purpose is to prove that an omitted old query cannot fail on
// finite-operation overflow/underflow or unresolved triangle degeneracy.
constexpr double Limit=0x1p100,SmallEdge=0x1p-100;
bool Eligible(const TriangleGeometry& triangle,double common_x) {
  for (const auto p:triangle.vertices)
    if (!IsFinite(p) || p.x!=common_x || geometry_detail::MaxAbs(p)>Limit) return false;
  for (unsigned i=0;i<3;++i) {
    const auto edge=Subtract(triangle.vertices[(i+1)%3],triangle.vertices[i]);
    if (!IsFinite(edge) || geometry_detail::MaxAbs(edge)<SmallEdge) return false;
  }
  const auto ab=Subtract(triangle.vertices[1],triangle.vertices[0]);
  const auto ac=Subtract(triangle.vertices[2],triangle.vertices[0]);
  const double scale=geometry_detail::Maximum(geometry_detail::MaxAbs(ab),geometry_detail::MaxAbs(ac));
  const auto u=geometry_detail::Divide(ab,scale),v=geometry_detail::Divide(ac,scale);
  const double determinant=u.y*v.z-u.z*v.y;
  // |u|_infinity,|v|_infinity<=1; largest_edge_squared<=8. This
  // gives an eightfold margin over 64eps*8, with rounding safety to spare.
  // On the exact common-X plane the cross has only one nonzero component;
  // both hypot(x,0) operations are exact abs(x). Either winding is legal.
  if (!IsFinite(determinant) || ::fabs(determinant)<4096*DBL_EPSILON) return false;
  TrianglePointGeometry original;
  return ClosestPointOnTriangle(triangle.vertices[0],triangle,&original)==Status::kOk && !original.degenerate;
}
} // namespace

Status PreparedPlanarWallQuery::Initialize(const planar_detail::WallFace* faces,
                                          std::uint32_t count,double tolerance) {
  if (!faces || !count || count>MaxPlanarWallTriangles) return Status::kInvalidArgument;
  PreparedPlanarWallQuery next;
  next.count_=count; next.tolerance_=tolerance; next.wall_x_=faces[0].geometry.vertices[0].x;
  next.eligible_=IsFinite(tolerance) && tolerance>=0 && tolerance<=Limit;
  for (std::uint32_t i=0;i<count;++i) {
    next.faces_[i]=faces[i];
    // Do not short-circuit copying at an ineligible/invalid late face. The
    // complete original sequence is required by fallback and its output rules.
    const auto& triangle=faces[i].geometry;
    if (!Eligible(triangle,next.wall_x_)) { next.eligible_=false; continue; }
    auto& box=next.boxes_[i];
    box.y_min=box.y_max=triangle.vertices[0].y;
    box.z_min=box.z_max=triangle.vertices[0].z;
    double magnitude=std::max(1.0,::fabs(tolerance));
    for (const auto p:triangle.vertices) {
      box.y_min=std::min(box.y_min,p.y); box.y_max=std::max(box.y_max,p.y);
      box.z_min=std::min(box.z_min,p.z); box.z_max=std::max(box.z_max,p.z);
      magnitude=std::max({magnitude,::fabs(p.y),::fabs(p.z)});
    }
    double padding=0,expanded=0;
    // Rounded convex-combination point <=12eps*M outside the vertex box;
    // coordinate subtraction plus two hypot calls require <=16eps*max(M,tol)
    // under the pinned numerical runtime assumptions. 64eps*M exceeds their
    // sum (including subnormals with M>=1); it is not fitted to source residuals.
    // Directed construction adds the original tolerance outward exactly once.
    if (!q4_bounds::MultiplyScalar(64*DBL_EPSILON,magnitude,true,&padding) ||
        !q4_bounds::AddScalar(tolerance,padding,true,&expanded) ||
        !q4_bounds::AddScalar(box.y_min,-expanded,false,&box.y_min) ||
        !q4_bounds::AddScalar(box.y_max,expanded,true,&box.y_max) ||
        !q4_bounds::AddScalar(box.z_min,-expanded,false,&box.z_min) ||
        !q4_bounds::AddScalar(box.z_max,expanded,true,&box.z_max)) next.eligible_=false;
  }
  next.prepared_=true;
  *this=next;
  return Status::kOk;
}
} // namespace tlfea::contact
