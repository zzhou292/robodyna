#include "PlanarWallBox.h"

namespace tlfea::contact {
namespace {
using Code=PlanarContactStatus;
PlanarContactReport Report(Code code,const char* message) { return {code,message,UINT32_MAX}; }

bool Expand(double physical_low,double physical_high,double minimum_span,
            double* query_low,double* query_high,double* lower_error,double* upper_error) {
  const double span=physical_high-physical_low;
  if (!IsFinite(span) || span < 0) return false;
  if (span >= minimum_span) return true;
  // One outward step encloses each correctly rounded endpoint operation. The
  // minimum span is a declared query scale, not an uncertain physical length.
  const double lo=::nextafter(physical_low-minimum_span,-HUGE_VAL);
  const double hi=::nextafter(physical_high+minimum_span,HUGE_VAL);
  if (!IsFinite(lo) || !IsFinite(hi) || !(lo < physical_low) || !(hi > physical_high)) return false;
  const double below=::nextafter(physical_low-lo,HUGE_VAL);
  const double above=::nextafter(hi-physical_high,HUGE_VAL);
  if (!IsFinite(below) || !IsFinite(above) || !(below > 0) || !(above > 0)) return false;
  *query_low=lo; *query_high=hi; *lower_error=below; *upper_error=above;
  return true;
}
} // namespace

PlanarContactReport CheckPlanarWallBox(
    const PlanarWallGeometry& wall,PlanarWallBox physical,double clearance,
    std::uint64_t feature_id,PlanarWallBoxMode mode,PlanarWallBoxCoverage* output) {
  if (!output) return Report(Code::InvalidOutput,"Missing planar box coverage output");
  if (!wall.initialized()) return Report(Code::NotInitialized,"Finite wall has not been prepared");
  if (!IsFinite(physical.minimum) || !IsFinite(physical.maximum) ||
      physical.minimum.x != wall.wall_x() || physical.maximum.x != wall.wall_x() ||
      physical.minimum.y > physical.maximum.y || physical.minimum.z > physical.maximum.z ||
      !feature_id || !IsFinite(clearance) || !(clearance > 8*wall.tolerance()) ||
      (mode != PlanarWallBoxMode::Exact && mode != PlanarWallBoxMode::ConservativeExpansion))
    return Report(Code::InvalidInput,"Invalid physical box, mode or finite-wall clearance");
  PlanarWallBoxCoverage next; next.physical=physical; next.query=physical; next.mode=mode;
  if (mode == PlanarWallBoxMode::ConservativeExpansion) {
    double coordinate_scale=1;
    const double values[6]={physical.minimum.y,physical.minimum.z,physical.maximum.y,physical.maximum.z,
                           physical.maximum.y-physical.minimum.y,physical.maximum.z-physical.minimum.z};
    for (double value:values) {
      if (!IsFinite(value)) return Report(Code::InvalidInput,"Projected box span is unrepresentable");
      if (::fabs(value) > coordinate_scale) coordinate_scale=::fabs(value);
    }
    // 64 times the larger of prepared-wall tolerance and a 128-epsilon local
    // coordinate scale. This comfortably exceeds the existing triangle query's
    // 64-epsilon aspect threshold; that query remains the final authority.
    const double roundoff_scale=128*DBL_EPSILON*coordinate_scale;
    const double minimum_span=64*(roundoff_scale > wall.tolerance() ? roundoff_scale : wall.tolerance());
    if (!IsFinite(minimum_span) || !(minimum_span > 0) ||
        !Expand(physical.minimum.y,physical.maximum.y,minimum_span,
                &next.query.minimum.y,&next.query.maximum.y,&next.lower_expansion_upper.y,&next.upper_expansion_upper.y) ||
        !Expand(physical.minimum.z,physical.maximum.z,minimum_span,
                &next.query.minimum.z,&next.query.maximum.z,&next.lower_expansion_upper.z,&next.upper_expansion_upper.z))
      return Report(Code::InvalidInput,"Conservative projected query expansion is unrepresentable");
  }
  // Exact mode intentionally retains the legacy C5a construction and order.
  const auto lo=next.query.minimum,hi=next.query.maximum;
  const Vec3 corner[4]={{lo.x,hi.y,hi.z},{lo.x,lo.y,hi.z},{lo.x,lo.y,lo.z},{lo.x,hi.y,lo.z}};
  constexpr unsigned triangles[2][3]={{0,1,2},{0,2,3}};
  for (const auto& nodes:triangles) {
    TriangleGeometry triangle; triangle.face_id=feature_id;
    for (unsigned n=0;n<3;++n) {
      triangle.vertices[n]=corner[nodes[n]];
      triangle.vertex_ids[n]=nodes[n];  // Local query keys, never source identities.
    }
    bool covered=false;
    const auto report=wall.ClassifyTriangle(triangle,clearance,&covered);
    if (report.status != Code::Ok) return report;
    if (!covered) return Report(Code::UnsupportedGeometry,"Swept projected box is outside the finite wall");
  }
  next.covered=true; *output=next;
  return Report(Code::Ok,"Swept projected box is wholly covered");
}
} // namespace tlfea::contact
