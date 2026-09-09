#include "PlanarMeshContactStorage.h"
#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <map>
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
struct Edge { std::uint32_t a = 0, b = 0, count = 0; int orientation = 0; };
using EdgeKey = std::pair<std::uint32_t, std::uint32_t>;
EdgeKey Key(std::uint32_t a, std::uint32_t b) { return std::minmax(a,b); }
bool HasNode(const PlanarWallTriangle& t, std::uint32_t node) {
  return t.nodes[0] == node || t.nodes[1] == node || t.nodes[2] == node;
}
SegmentGeometry Segment(Vec3 a, Vec3 b) { return {{a,b},{1,2}}; }
bool CloseSegments(const SegmentGeometry& a, const SegmentGeometry& b, double clearance) {
  SegmentPairGeometry pair;
  return ClosestPointsBetweenSegments(a,b,&pair) != Status::kOk || pair.distance <= clearance;
}

PlanarContactReport ValidateWall(PlanarWallView input, std::vector<WallFace>* faces,
                                 std::vector<Edge>* boundary, double* tolerance) {
  const double x = input.vertices[0].position.x;
  double scale = 1;
  std::set<std::uint64_t> source_nodes, assembled_nodes, face_ids;
  for (std::uint32_t i = 0; i < input.vertex_count; ++i) {
    const auto& v = input.vertices[i];
    if (!IsFinite(v.position) || v.position.x != x || !v.source_node_id || !v.assembled_source_node_id ||
        !source_nodes.insert(v.source_node_id).second || !assembled_nodes.insert(v.assembled_source_node_id).second)
      return Report(PStatus::UnsupportedGeometry,"Wall needs unique source IDs and exactly constant finite x");
    const auto delta = Subtract(v.position,input.vertices[0].position);
    if (!IsFinite(delta)) return Report(PStatus::InvalidInput,"Unrepresentable wall extent");
    scale = std::max({scale,std::abs(v.position.y),std::abs(v.position.z),
                      std::abs(delta.y),std::abs(delta.z)});
    for (std::uint32_t j = 0; j < i; ++j)
      if (v.position.y == input.vertices[j].position.y && v.position.z == input.vertices[j].position.z)
        return Report(PStatus::UnsupportedGeometry,"Coincident wall nodes must share one topology index");
  }
  *tolerance = 128 * DBL_EPSILON * scale;
  if (!IsFinite(*tolerance) || *tolerance <= 0)
    return Report(PStatus::InvalidInput,"Unrepresentable geometry tolerance");
  std::map<EdgeKey,Edge> edges;
  std::set<std::array<std::uint32_t,3>> connectivity;
  std::map<std::uint64_t,std::uint64_t> source_quads, assembled_quads;
  std::vector<bool> used(input.vertex_count,false);
  faces->resize(input.triangle_count);
  for (std::uint32_t i = 0; i < input.triangle_count; ++i) {
    const auto& t = input.triangles[i];
    if (!t.triangle_id || !t.source_quad_id || !t.assembled_source_quad_id || !face_ids.insert(t.triangle_id).second)
      return Report(PStatus::InvalidInput,"Missing or duplicate wall face identity");
    if (source_quads.emplace(t.source_quad_id,t.assembled_source_quad_id).first->second != t.assembled_source_quad_id ||
        assembled_quads.emplace(t.assembled_source_quad_id,t.source_quad_id).first->second != t.source_quad_id)
      return Report(PStatus::InvalidInput,"Inconsistent source/assembled wall parent identity");
    std::array<std::uint32_t,3> sorted{{t.nodes[0],t.nodes[1],t.nodes[2]}};
    std::sort(sorted.begin(),sorted.end());
    if (sorted[2] >= input.vertex_count || sorted[0] == sorted[1] || sorted[1] == sorted[2] ||
        !connectivity.insert(sorted).second)
      return Report(PStatus::InvalidInput,"Invalid or duplicate wall triangle connectivity");
    auto& face = (*faces)[i];
    face.geometry.face_id = t.triangle_id;
    face.source_quad_id = t.source_quad_id; face.assembled_source_quad_id = t.assembled_source_quad_id;
    for (int j = 0; j < 3; ++j) {
      const auto node = t.nodes[j]; used[node] = true;
      face.geometry.vertices[j] = input.vertices[node].position;
      face.geometry.vertex_ids[j] = input.vertices[node].assembled_source_node_id;
      const auto next = t.nodes[(j+1)%3];
      const auto key = Key(node,next);
      auto& edge = edges[key]; edge.a = key.first; edge.b = key.second;
      ++edge.count; edge.orientation += node < next ? 1 : -1;
    }
    TrianglePointGeometry result;
    if (ClosestPointOnTriangle(face.geometry.vertices[0],face.geometry,&result) != Status::kOk ||
        result.degenerate || result.face_normal.x != -1 || result.face_normal.y != 0 || result.face_normal.z != 0)
      return Report(PStatus::UnsupportedGeometry,"Wall triangles must be nondegenerate and wound toward -X");
  }
  for (bool present : used)
    if (!present) return Report(PStatus::InvalidInput,"Unused wall vertex");
  for (const auto& item : edges) {
    const auto& edge = item.second;
    if (edge.count > 2 || (edge.count == 2 && edge.orientation != 0))
      return Report(PStatus::UnsupportedGeometry,"Nonmanifold or inconsistent wall edge");
    if (edge.count == 1) boundary->push_back(edge);
  }
  if (boundary->empty()) return Report(PStatus::UnsupportedGeometry,"Finite planar wall has no exposed boundary");
  // Validate the WHOLE supplied mesh, not only faces beneath samples. These
  // bounded startup checks reject overlap, crossing edges and T junctions.
  for (std::uint32_t i = 0; i < input.triangle_count; ++i) {
    for (std::uint32_t j = 0; j < i; ++j) {
      const auto& a = input.triangles[i]; const auto& b = input.triangles[j];
      for (int side = 0; side < 2; ++side) {
        const auto& from = side ? b : a; const auto& into = side ? a : b;
        const auto& into_geometry = (*faces)[side ? i : j].geometry;
        for (auto node : from.nodes) if (!HasNode(into,node)) {
          TrianglePointGeometry result;
          if (ClosestPointOnTriangle(input.vertices[node].position,into_geometry,&result) != Status::kOk ||
              result.distance <= *tolerance)
            return Report(PStatus::UnsupportedGeometry,"Overlapping wall interiors or nonconforming vertex/edge");
        }
      }
      for (int ea = 0; ea < 3; ++ea) for (int eb = 0; eb < 3; ++eb) {
        const auto a0 = a.nodes[ea], a1 = a.nodes[(ea+1)%3];
        const auto b0 = b.nodes[eb], b1 = b.nodes[(eb+1)%3];
        if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1) continue;
        if (CloseSegments(Segment(input.vertices[a0].position,input.vertices[a1].position),
                          Segment(input.vertices[b0].position,input.vertices[b1].position),*tolerance))
          return Report(PStatus::UnsupportedGeometry,"Crossing or unresolved wall edges");
      }
    }
  }
  return Report(PStatus::Ok,"Wall validated");
}

PlanarContactReport ValidateSurface(const PlanarContactConfig& config, PlanarSurfaceView surface,
                                    PlanarWallView wall, const std::vector<WallFace>& faces,
                                    const std::vector<Edge>& boundary, double tolerance,
                                    std::vector<SurfacePoint>* points) {
  if (!(config.exposed_boundary_clearance_m > 8*tolerance))
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
      projected.vertices[j].x = wall.vertices[0].position.x;
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
    for (int j = 0; j < 4; ++j) {
      Vec3 query = j < 3 ? projected.vertices[j] :
          Add(Add(Scale(projected.vertices[0],1.0/3),Scale(projected.vertices[1],1.0/3)),
              Scale(projected.vertices[2],1.0/3));
      query.x = wall.vertices[0].position.x;
      std::uint32_t owner; TrianglePointGeometry closest;
      if (planar_detail::FindOwner(query,faces.data(),wall.triangle_count,tolerance,&owner,&closest) != Status::kOk)
        return Report(PStatus::InvalidInput,"Unrepresentable finite-wall coverage",i);
      const bool here = owner != UINT32_MAX;
      if (j == 0) covered = here;
      else if (covered != here) return Report(PStatus::AmbiguousBoundary,"Surface crosses finite-wall footprint",i);
    }
    for (const auto& edge : boundary) {
      const auto a = wall.vertices[edge.a].position, b = wall.vertices[edge.b].position;
      for (int j = 0; j < 3; ++j)
        if (CloseSegments(Segment(projected.vertices[j],projected.vertices[(j+1)%3]),Segment(a,b),
                          config.exposed_boundary_clearance_m))
          return Report(PStatus::AmbiguousBoundary,"Surface is too close to an exposed wall edge",i);
      // Also exclude a hole or isolated wall component enclosed by a large
      // surface triangle, whose boundary could miss all three surface edges.
      for (const auto endpoint : {a,b}) {
        TrianglePointGeometry closest;
        if (ClosestPointOnTriangle(endpoint,projected,&closest) != Status::kOk ||
            closest.distance <= config.exposed_boundary_clearance_m)
          return Report(PStatus::AmbiguousBoundary,"Surface encloses or touches a wall boundary",i);
      }
    }
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
    std::vector<WallFace> faces; std::vector<Edge> boundary; std::vector<SurfacePoint> points;
    auto report = ValidateWall(wall,&faces,&boundary,&next->geometry_tolerance);
    if (report.status != PStatus::Ok) return report;
    report = ValidateSurface(config,surface,wall,faces,boundary,next->geometry_tolerance,&points);
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
        (error=cudaMemcpy(next->wall,faces.data(),wall_bytes,cudaMemcpyHostToDevice)) != cudaSuccess ||
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
