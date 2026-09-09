#include "PlanarMeshContactStorage.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <new>
#include <set>
#include <utility>
#include <vector>

namespace tlfea::contact {
namespace {
using planar_detail::WallFace;
using planar_detail::SurfacePoint;
using PStatus = PlanarContactStatus;
PlanarContactReport Report(PStatus status, const char* message, std::uint32_t sample = UINT32_MAX) {
  return {status, message, sample};
}
PlanarContactReport ValidateSurface(const PlanarContactConfig& config, PlanarSurfaceView surface,
                                    const PlanarWallGeometry& wall,
                                    std::vector<SurfacePoint>* points) {
  if (!(config.exposed_boundary_clearance_m > 8*wall.tolerance()))
    return Report(PStatus::InvalidInput,"Boundary clearance must exceed geometric roundoff band");
  std::set<std::uint32_t> global_nodes;
  std::set<std::uint64_t> source_nodes, face_ids;
  std::vector<bool> used(surface.node_count,false);
  for (std::uint32_t i = 0; i < surface.node_count; ++i) {
    const auto& node = surface.nodes[i];
    if (node.global_node >= config.global_node_count || !IsFinite(node.reference_position) ||
        node.reference_position.x != surface.nodes[0].reference_position.x ||
        !node.source_node_id || !global_nodes.insert(node.global_node).second ||
        !source_nodes.insert(node.source_node_id).second)
      return Report(PStatus::InvalidInput,"Invalid surface-node mapping");
  }
  std::set<std::array<std::uint32_t,3>> connectivity;
  points->resize(surface.triangle_count);
  for (std::uint32_t i = 0; i < surface.triangle_count; ++i) {
    const auto& input = surface.triangles[i];
    std::array<std::uint32_t,3> sorted{{input.local_nodes[0],input.local_nodes[1],input.local_nodes[2]}};
    std::sort(sorted.begin(),sorted.end());
    if (!input.triangle_id || !face_ids.insert(input.triangle_id).second || sorted[2] >= surface.node_count ||
        sorted[0] == sorted[1] || sorted[1] == sorted[2] || !connectivity.insert(sorted).second)
      return Report(PStatus::InvalidInput,"Invalid surface face identity/connectivity",i);
    auto& point = (*points)[i];
    point.triangle.feature_id = input.triangle_id;
    point.triangle.parent_element_id = input.triangle_id;
    point.triangle.interpolation = SurfaceInterpolation::kLinearTriangle;
    TriangleGeometry projected;
    projected.face_id = input.triangle_id;
    for (int j = 0; j < 3; ++j) {
      const auto local = input.local_nodes[j]; used[local] = true;
      point.local_nodes[j] = local;
      point.triangle.nodes[j] = surface.nodes[local].global_node;
      projected.vertices[j] = surface.nodes[local].reference_position;
      projected.vertices[j].x = wall.wall_x();
      projected.vertex_ids[j] = surface.nodes[local].source_node_id;
    }
    TrianglePointGeometry check;
    if (ClosestPointOnTriangle(projected.vertices[0],projected,&check) != Status::kOk || check.degenerate)
      return Report(PStatus::UnsupportedGeometry,"Surface has degenerate normal projection",i);
    const auto ab = Subtract(projected.vertices[1],projected.vertices[0]);
    const auto ac = Subtract(projected.vertices[2],projected.vertices[0]);
    point.area = .5 * std::abs(ab.y*ac.z-ab.z*ac.y);
    point.stiffness = config.penalty_per_area*point.area;
    if (!IsFinite(point.area) || point.area <= 0 || !IsFinite(point.stiffness) || point.stiffness <= 0)
      return Report(PStatus::InvalidInput,"Unrepresentable surface area/stiffness",i);
    bool covered = false;
    auto report = wall.ClassifyTriangle(projected,config.exposed_boundary_clearance_m,&covered);
    if (report.status != PStatus::Ok) { report.sample = i; return report; }
    point.covered = covered;
  }
  for (bool present : used)
    if (!present) return Report(PStatus::InvalidInput,"Unused surface node");
  return Report(PStatus::Ok,"Surface validated");
}
}  // namespace

PlanarMeshContact::PlanarMeshContact() = default;
PlanarMeshContact::~PlanarMeshContact() = default;
PlanarMeshContact::Impl::~Impl() {
  if (control) cudaFree(control);
  if (points) cudaFree(points);
  if (nodes) cudaFree(nodes);
  if (wall) cudaFree(wall);
}
PlanarContactReport PlanarMeshContact::Initialize(const PlanarContactConfig& config,
                                                  PlanarWallView wall, PlanarSurfaceView surface) {
  if (impl_) return Report(PStatus::InvalidInput,"Contact batch already initialized");
  if (!wall.vertex_count || wall.vertex_count > MaxPlanarWallVertices || !wall.triangle_count ||
      wall.triangle_count > MaxPlanarWallTriangles || !surface.node_count || surface.node_count > MaxPlanarSurfaceNodes ||
      !surface.triangle_count || surface.triangle_count > MaxPlanarSurfaceTriangles ||
      !config.global_node_count || config.global_node_count > tl::fea::MaxTranslationNodes ||
      !config.max_device_bytes || config.max_device_bytes > MaxPlanarContactDeviceBytes)
    return Report(PStatus::ResourceLimit,"Contact capacity exceeds admitted bounds");
  if (!config.owner_id || !wall.vertices || !wall.triangles || !surface.nodes || !surface.triangles ||
      !IsFinite(config.penalty_per_area) || config.penalty_per_area <= 0 ||
      !IsFinite(config.exposed_boundary_clearance_m) || config.exposed_boundary_clearance_m <= 0 ||
      !IsFinite(config.max_penetration_m) || config.max_penetration_m <= 0)
    return Report(PStatus::InvalidInput,"Invalid planar contact configuration");
  const auto wall_bytes = sizeof(WallFace)*wall.triangle_count;
  const auto node_bytes = sizeof(PlanarSurfaceNode)*surface.node_count;
  const auto point_bytes = sizeof(SurfacePoint)*surface.triangle_count;
  const auto total_bytes = wall_bytes+node_bytes+point_bytes+sizeof(planar_detail::Control);
  if (total_bytes > config.max_device_bytes) return Report(PStatus::ResourceLimit,"Contact device budget is insufficient");
  try {
    auto next = std::make_unique<Impl>(); next->config = config;
    next->wall_count = wall.triangle_count; next->node_count = surface.node_count; next->point_count = surface.triangle_count;
    PlanarWallGeometry geometry;
    std::vector<SurfacePoint> points;
    auto report = geometry.Initialize(wall);
    if (report.status != PStatus::Ok) return report;
    next->geometry_tolerance = geometry.tolerance();
    report = ValidateSurface(config,surface,geometry,&points);
    if (report.status != PStatus::Ok) return report;
    next->wall_x = wall.vertices[0].position.x;
    auto allocate = [&](auto** pointer, std::size_t bytes) {
      const auto error = cudaMalloc(reinterpret_cast<void**>(pointer),bytes);
      if (error == cudaSuccess) { next->allocation.device_bytes += bytes; ++next->allocation.device_allocations; }
      return error;
    };
    cudaError_t error;
    if ((error=allocate(&next->wall,wall_bytes)) != cudaSuccess ||
        (error=allocate(&next->nodes,node_bytes)) != cudaSuccess ||
        (error=allocate(&next->points,point_bytes)) != cudaSuccess ||
        (error=allocate(&next->control,sizeof(planar_detail::Control))) != cudaSuccess ||
        (error=cudaMemcpy(next->wall,geometry.faces().data(),wall_bytes,cudaMemcpyHostToDevice)) != cudaSuccess ||
        (error=cudaMemcpy(next->nodes,surface.nodes,node_bytes,cudaMemcpyHostToDevice)) != cudaSuccess ||
        (error=cudaMemcpy(next->points,points.data(),point_bytes,cudaMemcpyHostToDevice)) != cudaSuccess ||
        (error=cudaMemcpy(next->control,&next->host_control,sizeof(planar_detail::Control),cudaMemcpyHostToDevice)) != cudaSuccess)
      return Report(PStatus::DeviceFailure,cudaGetErrorString(error));
    impl_ = std::move(next); return Report(PStatus::Ok,"Planar contact initialized");
  } catch (const std::bad_alloc&) { return Report(PStatus::ResourceLimit,"Contact startup allocation failed"); }
}
const PlanarContactDiagnostics* PlanarMeshContact::diagnostics() const noexcept {
  return impl_ && impl_->host_control.diagnostics.valid ? &impl_->host_control.diagnostics : nullptr;
}
PlanarContactAllocationInfo PlanarMeshContact::allocations() const noexcept {
  return impl_ ? impl_->allocation : PlanarContactAllocationInfo{};
}
}  // namespace tlfea::contact
