#include "Q4PlanarGeometry.h"

#include "PlanarWallGeometry.h"
#include "Q4ContactBounds.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace tlfea::contact {
namespace {
using PStatus=PlanarContactStatus;
PlanarContactReport Report(PStatus status,const char* message,std::uint32_t parent=UINT32_MAX) {
  return {status,message,parent};
}
PlanarContactReport PrepareRectangle(const PlanarWallGeometry& wall,const Q4SurfaceView& input,
                                    std::uint32_t parent_index,PreparedQ4PlanarParent* output) {
  const auto& parent=input.parents[parent_index];
  if (q4_detail::ValidateParent(parent,input.positions.node_count) != Status::kOk)
    return Report(PStatus::UnsupportedGeometry,"Invalid Q4 parent identity, connectivity or offset",parent_index);
  PreparedQ4PlanarParent candidate; candidate.parent=parent;
  double scale=1;
  for (unsigned i=0;i<4;++i) {
    const auto position=input.positions.at(parent.nodes[i]);
    if (!IsFinite(position)) return Report(PStatus::InvalidInput,"Nonfinite Q4 reference position",parent_index);
    candidate.reference_projection[i]={wall.wall_x(),position.y,position.z};
    scale=std::max({scale,std::abs(position.y),std::abs(position.z)});
  }
  const auto* p=candidate.reference_projection;
  if (p[0].y != p[3].y || p[1].y != p[2].y || p[0].z != p[1].z || p[2].z != p[3].z ||
      !(p[0].y > p[1].y) || !(p[0].z > p[3].z))
    return Report(PStatus::UnsupportedGeometry,"Q4 projection must be an axis-aligned YZ rectangle in natural order",parent_index);
  const auto side_y=Subtract(p[0],p[1]),side_z=Subtract(p[0],p[3]);
  if (!IsFinite(side_y) || !IsFinite(side_z))
    return Report(PStatus::InvalidInput,"Unrepresentable Q4 projected side",parent_index);
  const double shorter=std::min(side_y.y,side_z.z),longer=std::max(side_y.y,side_z.z);
  scale=std::max(scale,longer);
  const double roundoff=std::max(wall.tolerance(),128*DBL_EPSILON*scale);
  if (!IsFinite(roundoff) || !(shorter > MinQ4SideRoundoffMultiple*roundoff) ||
      !IsFinite(longer/shorter) || longer/shorter > MaxQ4ProjectedAspectRatio)
    return Report(PStatus::UnsupportedGeometry,"Q4 projected sides exceed the declared resolution/aspect envelope",parent_index);
  candidate.projected_area=geometry_detail::Cross(side_y,side_z).x;
  Q4IntegralInterval width,height;
  if (!IsFinite(candidate.projected_area) || candidate.projected_area <= 0 ||
      !q4_bounds::Difference(p[0].y,p[1].y,&width) || !q4_bounds::Difference(p[0].z,p[3].z,&height) ||
      !q4_bounds::MultiplyPositive(width,height,&candidate.area_enclosure) ||
      candidate.projected_area < candidate.area_enclosure.lower || candidate.projected_area > candidate.area_enclosure.upper)
    return Report(PStatus::InvalidInput,"Unrepresentable Q4 projected area or area enclosure",parent_index);
  *output=candidate;
  return Report(PStatus::Ok,"Q4 rectangle prepared",parent_index);
}
bool InteriorsOverlap(const PreparedQ4PlanarParent& a,const PreparedQ4PlanarParent& b) {
  const auto* x=a.reference_projection; const auto* y=b.reference_projection;
  return std::max(x[1].y,y[1].y) < std::min(x[0].y,y[0].y) &&
         std::max(x[3].z,y[3].z) < std::min(x[0].z,y[0].z);
}
PlanarContactReport ClassifyRectangle(const PlanarWallGeometry& wall,double clearance,
                                     std::uint32_t parent_index,PreparedQ4PlanarParent* output) {
  // Both triangulations are coverage oracles only. Natural Q4 interpolation,
  // source parent identity, area and every mechanical quantity are unchanged.
  const unsigned triangles[4][3]={{0,1,2},{0,2,3},{0,1,3},{1,2,3}};
  bool covered=false;
  for (unsigned side=0;side<4;++side) {
    TriangleGeometry triangle; triangle.face_id=output->parent.feature_id;
    for (unsigned i=0;i<3;++i) {
      const auto local=triangles[side][i];
      triangle.vertices[i]=output->reference_projection[local];
      // Local physical keys are used only within this temporary triangle's
      // geometry query. They are not invented global source-node identities.
      triangle.vertex_ids[i]=output->parent.nodes[local];
    }
    bool here=false;
    auto report=wall.ClassifyTriangle(triangle,clearance,&here);
    if (report.status != PStatus::Ok) { report.sample=parent_index; return report; }
    if (side == 0) covered=here;
    else if (covered != here)
      return Report(PStatus::AmbiguousBoundary,"Q4 display partitions disagree on finite-wall coverage",parent_index);
  }
  output->covered=covered;
  return Report(PStatus::Ok,"Q4 footprint classified",parent_index);
}
}  // namespace

PlanarContactReport Q4PlanarGeometry::Initialize(const PlanarWallGeometry& wall,const Q4SurfaceView& reference,
                                               const Q4FixedYZMassView& mass,double clearance) {
  if (initialized()) return Report(PStatus::InvalidInput,"Q4 planar geometry is already initialized");
  if (!wall.initialized()) return Report(PStatus::NotInitialized,"Finite wall has not been prepared");
  if (!reference.parent_count || reference.parent_count > MaxQ4PlanarParents)
    return Report(PStatus::ResourceLimit,"Q4 planar parent count is outside the two-parent gate");
  if (!reference.parents || !reference.positions.valid() || !reference.velocities.valid() ||
      reference.positions.node_count != reference.velocities.node_count || mass.node_count != reference.positions.node_count ||
      !IsFinite(clearance) || !(clearance > 8*wall.tolerance()))
    return Report(PStatus::InvalidInput,"Invalid Q4 views, mass extent or finite-wall clearance");
  std::array<PreparedQ4PlanarParent,MaxQ4PlanarParents> candidate{};
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto report=PrepareRectangle(wall,reference,p,&candidate[p]);
    if (report.status != PStatus::Ok) return report;
    for (std::uint32_t previous=0;previous<p;++previous) {
      if (candidate[p].parent.feature_id == candidate[previous].parent.feature_id ||
          candidate[p].parent.parent_element_id == candidate[previous].parent.parent_element_id)
        return Report(PStatus::InvalidInput,"Duplicate Q4 feature or parent identity",p);
      if (InteriorsOverlap(candidate[p],candidate[previous]))
        return Report(PStatus::UnsupportedGeometry,"Q4 projected interiors overlap regardless of physical node identity",p);
    }
  }
  const Q4PlanarReferenceView prepared{candidate.data(),reference.parent_count,reference.positions.node_count,
                                      wall.wall_x(),wall.tolerance()};
  const auto motion=ValidateQ4PlanarMotion(prepared,reference,mass);
  if (motion != PStatus::Ok) return Report(motion,"Q4 reference motion, mapping or component mass is unsupported");
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto report=ClassifyRectangle(wall,clearance,p,&candidate[p]);
    if (report.status != PStatus::Ok) return report;
  }
  parents_=candidate; parent_count_=reference.parent_count; global_node_count_=reference.positions.node_count;
  wall_x_=wall.wall_x(); wall_tolerance_=wall.tolerance();
  return Report(PStatus::Ok,"Q4 planar geometry initialized");
}
}  // namespace tlfea::contact
