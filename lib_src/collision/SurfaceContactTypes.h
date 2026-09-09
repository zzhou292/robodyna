#pragma once

#include <cfloat>
#include <cmath>
#include <cstdint>

// Small arithmetic and borrowed views shared by host regression tests and future
// CUDA contact kernels. No allocation, CUDA runtime, or independent time advance.
#if defined(__CUDACC__)
#define TL_SURFACE_HD __host__ __device__
#else
#define TL_SURFACE_HD
#endif

namespace tlfea {
namespace contact {

enum class Status : std::uint8_t {
  kOk,
  kInvalidArgument,
  kOutOfRange,
  kNonFiniteResult,
  kNoDynamicDofs,
  kStaleTrial,
  kNoTrial,
  kUnsupportedInterpolation,
};

struct Vec3 {
  double x = 0;
  double y = 0;
  double z = 0;
};

TL_SURFACE_HD inline bool IsFinite(double a) {
  return a == a && a <= DBL_MAX && a >= -DBL_MAX;
}
TL_SURFACE_HD inline bool IsFinite(Vec3 a) {
  return IsFinite(a.x) && IsFinite(a.y) && IsFinite(a.z);
}
TL_SURFACE_HD inline Vec3 Add(Vec3 a, Vec3 b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
TL_SURFACE_HD inline Vec3 Subtract(Vec3 a, Vec3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
TL_SURFACE_HD inline Vec3 Scale(Vec3 a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}
TL_SURFACE_HD inline double Dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Strides count doubles. Supports existing TL SoA positions (node_stride=1,
// component_stride=node_count) and AoS velocities (node_stride=3,
// component_stride=1) without copying or silently assuming the same layout.
// Pointers must all belong to the execution memory space; these views do not
// validate allocation lengths or synchronize streams. The caller owns lifetime.
struct VectorView {
  const double* data = nullptr;
  std::uint32_t node_count = 0;
  std::uint64_t node_stride = 0;
  std::uint64_t component_stride = 0;

  TL_SURFACE_HD bool valid() const {
    if (!data || node_count == 0 || node_stride == 0 || component_stride == 0)
      return false;
    const auto limit = UINT64_MAX / sizeof(double);
    return component_stride <= limit / 2 &&
           (node_count == 1 ||
            node_stride <= (limit - 2 * component_stride) / (node_count - 1));
  }

  // Call only after validating this view and the node index.
  TL_SURFACE_HD Vec3 at(std::uint32_t node) const {
    const auto offset = static_cast<std::uint64_t>(node) * node_stride;
    return {data[offset], data[offset + component_stride],
            data[offset + 2 * component_stride]};
  }
};

enum class SurfaceInterpolation : std::uint8_t {
  kUnspecified,
  kLinearTriangle,
};

struct SurfaceTriangle {
  std::uint32_t nodes[3] = {0, 0, 0};
  std::uint64_t feature_id = 0;        // Stable across contact trial evaluations.
  std::uint64_t parent_element_id = 0;
  std::uint32_t parent_face_id = 0;
  double half_thickness = 0;           // Geometry offset only, in metres.
  SurfaceInterpolation interpolation = SurfaceInterpolation::kUnspecified;
};

// A triangle is a physical linear FE face here, not merely a tessellated corner
// proxy for a curved ANCF face. ANCF requires its own J(q) and gradient-DOF
// scatter. Parent IDs preserve the route to that later formulation adapter.
struct LinearTrianglePoint {
  std::uint32_t triangle_index = 0;
  double weights[3] = {1, 0, 0};       // Barycentric shape values at contact.
};

struct LinearTriangleSurfaceView {
  VectorView positions;
  VectorView velocities;              // Required; no rigid-motion fallback.
  const double* inverse_node_mass = nullptr;  // kg^-1, isotropic lumped mass.
  const SurfaceTriangle* triangles = nullptr;
  std::uint32_t triangle_count = 0;
};

struct LinearPointKinematics {
  Vec3 position;                     // Interpolated midsurface location.
  Vec3 velocity;
  double inverse_effective_mass = 0;  // J M^-1 J^T for a unit direction.
};

TL_SURFACE_HD inline Status ValidatePoint(
    const LinearTriangleSurfaceView& surface, const LinearTrianglePoint& point) {
  if (!surface.positions.valid() || !surface.velocities.valid() ||
      surface.positions.node_count != surface.velocities.node_count ||
      !surface.inverse_node_mass || !surface.triangles)
    return Status::kInvalidArgument;
  if (point.triangle_index >= surface.triangle_count)
    return Status::kOutOfRange;
  double sum = 0;
  const auto& triangle = surface.triangles[point.triangle_index];
  if (triangle.interpolation != SurfaceInterpolation::kLinearTriangle)
    return Status::kUnsupportedInterpolation;
  if (!IsFinite(triangle.half_thickness) || triangle.half_thickness < 0)
    return Status::kInvalidArgument;
  for (int i = 0; i < 3; ++i) {
    if (!IsFinite(point.weights[i]) || point.weights[i] < 0 || point.weights[i] > 1)
      return Status::kInvalidArgument;
    if (triangle.nodes[i] >= surface.positions.node_count)
      return Status::kOutOfRange;
    for (int j = 0; j < i; ++j)
      if (triangle.nodes[i] == triangle.nodes[j])
        return Status::kInvalidArgument;
    sum += point.weights[i];
  }
  // Do not silently clamp or renormalize coordinates from narrowphase.
  return ::fabs(sum - 1) <= 1e-12 ? Status::kOk : Status::kInvalidArgument;
}

// Thickness is carried as geometry metadata, not applied here. A later offset
// surface adapter must provide its own Jacobian (including changes in normal)
// before using tangential forces on a finite-thickness shell skin.
TL_SURFACE_HD inline Status EvaluateLinearPoint(
    const LinearTriangleSurfaceView& surface, const LinearTrianglePoint& point,
    LinearPointKinematics* out) {
  if (!out)
    return Status::kInvalidArgument;
  *out = {};
  const auto status = ValidatePoint(surface, point);
  if (status != Status::kOk)
    return status;
  LinearPointKinematics value;
  const auto& triangle = surface.triangles[point.triangle_index];
  for (int i = 0; i < 3; ++i) {
    const auto node = triangle.nodes[i];
    const double inverse_mass = surface.inverse_node_mass[node];
    const Vec3 x = surface.positions.at(node);
    const Vec3 v = surface.velocities.at(node);
    if (!IsFinite(x) || !IsFinite(v) || !IsFinite(inverse_mass) || inverse_mass < 0)
      return Status::kInvalidArgument;
    const double weight = point.weights[i];
    value.position = Add(value.position, Scale(x, weight));
    value.velocity = Add(value.velocity, Scale(v, weight));
    value.inverse_effective_mass += weight * weight * inverse_mass;
  }
  if (!IsFinite(value.position) || !IsFinite(value.velocity) ||
      !IsFinite(value.inverse_effective_mass))
    return Status::kNonFiniteResult;
  *out = value;
  return Status::kOk;
}

struct TriangleNodalForces {
  std::uint32_t nodes[3] = {0, 0, 0};
  Vec3 forces[3];
};

// Returns J^T f to the caller's scatter kernel. No non-atomic writes to shared
// global node buffers. The contact-point velocity above uses the SAME J.
TL_SURFACE_HD inline Status ProjectLinearPointForce(
    const LinearTriangleSurfaceView& surface, const LinearTrianglePoint& point,
    Vec3 force, TriangleNodalForces* out) {
  if (!out)
    return Status::kInvalidArgument;
  *out = {};
  const auto status = ValidatePoint(surface, point);
  if (status != Status::kOk)
    return status;
  if (!IsFinite(force))
    return Status::kInvalidArgument;
  const auto& triangle = surface.triangles[point.triangle_index];
  for (int i = 0; i < 3; ++i) {
    out->nodes[i] = triangle.nodes[i];
    out->forces[i] = Scale(force, point.weights[i]);
  }
  return Status::kOk;
}

}  // namespace contact
}  // namespace tlfea
