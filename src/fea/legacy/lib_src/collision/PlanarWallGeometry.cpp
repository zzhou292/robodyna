#include "PlanarWallGeometry.h"
#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <map>
#include <new>
#include <set>
#include <utility>

namespace tlfea::contact {
namespace {
using planar_detail::WallFace;
using PStatus = PlanarContactStatus;
PlanarContactReport Report(PStatus status, const char* message) { return {status,message,UINT32_MAX}; }
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

}  // namespace

PlanarContactReport PlanarWallGeometry::Initialize(PlanarWallView input) {
  if (initialized_) return Report(PStatus::InvalidInput,"Wall geometry already initialized");
  if (!input.vertex_count || input.vertex_count > MaxPlanarWallVertices ||
      !input.triangle_count || input.triangle_count > MaxPlanarWallTriangles)
    return Report(PStatus::ResourceLimit,"Wall capacity exceeds admitted bounds");
  if (!input.vertices || !input.triangles)
    return Report(PStatus::InvalidInput,"Missing wall input");
  try {
    std::vector<WallFace> faces;
    std::vector<Edge> edges;
    double tolerance = 0;
    const auto report = ValidateWall(input,&faces,&edges,&tolerance);
    if (report.status != PStatus::Ok) return report;
    std::vector<planar_detail::BoundarySegment> boundary;
    boundary.reserve(edges.size());
    for (const auto& edge : edges)
      boundary.push_back({input.vertices[edge.a].position,input.vertices[edge.b].position});
    faces_ = std::move(faces); boundary_ = std::move(boundary);
    tolerance_ = tolerance; wall_x_ = input.vertices[0].position.x;
    initialized_ = true;
    return Report(PStatus::Ok,"Wall geometry initialized");
  } catch (const std::bad_alloc&) { return Report(PStatus::ResourceLimit,"Wall startup allocation failed"); }
}

PlanarContactReport PlanarWallGeometry::ClassifyTriangle(const TriangleGeometry& projected,
                                                       double clearance, bool* output) const {
  if (!initialized_) return Report(PStatus::NotInitialized,"Wall geometry is not initialized");
  if (!output || !IsFinite(clearance) || !(clearance > 8*tolerance_))
    return Report(PStatus::InvalidInput,"Boundary clearance must exceed geometric roundoff band");
  for (const auto vertex : projected.vertices)
    if (!IsFinite(vertex) || vertex.x != wall_x_)
      return Report(PStatus::UnsupportedGeometry,"Footprint must be finite and exactly on the wall plane");
  TrianglePointGeometry check;
  if (ClosestPointOnTriangle(projected.vertices[0],projected,&check) != Status::kOk || check.degenerate)
    return Report(PStatus::UnsupportedGeometry,"Surface has degenerate normal projection");
  bool covered = false;
  for (int j = 0; j < 4; ++j) {
    Vec3 query = j < 3 ? projected.vertices[j] :
        Add(Add(Scale(projected.vertices[0],1.0/3),Scale(projected.vertices[1],1.0/3)),
            Scale(projected.vertices[2],1.0/3));
    query.x = wall_x_;
    std::uint32_t owner; TrianglePointGeometry closest;
    if (planar_detail::FindOwner(query,faces_.data(),static_cast<std::uint32_t>(faces_.size()),
                                 tolerance_,&owner,&closest) != Status::kOk)
      return Report(PStatus::InvalidInput,"Unrepresentable finite-wall coverage");
    const bool here = owner != UINT32_MAX;
    if (j == 0) covered = here;
    else if (covered != here) return Report(PStatus::AmbiguousBoundary,"Surface crosses finite-wall footprint");
  }
  for (const auto& edge : boundary_) {
    for (int j = 0; j < 3; ++j)
      if (CloseSegments(Segment(projected.vertices[j],projected.vertices[(j+1)%3]),Segment(edge.a,edge.b),clearance))
        return Report(PStatus::AmbiguousBoundary,"Surface is too close to an exposed wall edge");
    // Retain the existing hole/isolated-component containment check even when
    // all three surface edges miss the exposed boundary.
    for (const auto endpoint : {edge.a,edge.b}) {
      TrianglePointGeometry closest;
      if (ClosestPointOnTriangle(endpoint,projected,&closest) != Status::kOk || closest.distance <= clearance)
        return Report(PStatus::AmbiguousBoundary,"Surface encloses or touches a wall boundary");
    }
  }
  *output = covered;
  return Report(PStatus::Ok,"Footprint classified");
}
}  // namespace tlfea::contact
