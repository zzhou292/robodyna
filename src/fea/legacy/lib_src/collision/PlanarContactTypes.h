#pragma once

#include "SurfaceContactTypes.h"
#include <cstddef>
#include <cstdint>

namespace tlfea::contact {
// Shared finite planar-wall input and reporting contracts. These POD views do
// not depend on CUDA ownership or a particular FE contact interpolation.
constexpr std::uint32_t MaxPlanarWallVertices = 1024, MaxPlanarWallTriangles = 512;
constexpr std::size_t MaxPlanarContactDeviceBytes = 1024 * 1024;

struct PlanarWallVertex {
  Vec3 position;
  std::uint64_t source_node_id = 0, assembled_source_node_id = 0;
};
struct PlanarWallTriangle {
  std::uint32_t nodes[3]{};
  std::uint64_t triangle_id = 0, source_quad_id = 0, assembled_source_quad_id = 0;
};
struct PlanarWallView {
  const PlanarWallVertex* vertices = nullptr;
  std::uint32_t vertex_count = 0;
  const PlanarWallTriangle* triangles = nullptr;
  std::uint32_t triangle_count = 0;
};
enum class PlanarContactStatus {
  Ok, InvalidInput, ResourceLimit, UnsupportedGeometry, UnsupportedMotion,
  AmbiguousBoundary, InvalidOutput, NotInitialized, WrongOwner, StaleAttempt,
  DeviceFailure
};
struct PlanarContactReport {
  PlanarContactStatus status = PlanarContactStatus::InvalidInput;
  const char* message = "Invalid request";
  std::uint32_t sample = UINT32_MAX;
};
}  // namespace tlfea::contact
