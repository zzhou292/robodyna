// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SelfContactFilterCertificates.h"
#include <cstddef>
#include <cstdint>

namespace tlfea::contact::self_contact_filters {
// Compact copied geometry only. Source identities and physical authority stay
// with the caller; these records never authorize activity, forces or a step.
struct TriangleGeometry { Vec3 vertices[3]; };
struct FacetProperties {
  double half_thickness = 0;
  std::uint32_t complete_rigid_group = UINT32_MAX;
};
struct SceneView {
  const TriangleGeometry* accepted = nullptr;
  const TriangleGeometry* prepared = nullptr;
  const FacetProperties* properties = nullptr;
  std::size_t count = 0;
};
struct PairView { const FixedTrianglePair* data = nullptr; std::size_t count = 0; };
struct PairResult {
  SelfContactFacetFilterStatus status = SelfContactFacetFilterStatus::InvalidInput;
  SelfContactFacetFilterCategory category = SelfContactFacetFilterCategory::ExactRemaining;
  bool separated = false;
  SelfContactFacetPrismSeparationAxis axis = SelfContactFacetPrismSeparationAxis::None;
};
struct ResultView {
  const PairResult* data = nullptr;
  std::size_t count = 0;
  std::uint64_t scene_generation = 0;
  bool complete = false;
};
enum class Status : std::uint8_t {
  Ok, InvalidInput, NotInitialized, AlreadyInitialized, ResourceLimit,
  NoScene, DeviceFailure, UnsupportedEnvironment,
};
struct Report {
  Status status = Status::Ok;
  const char* message = "OK";
  std::size_t pair = SIZE_MAX;
};
struct Limits {
  std::size_t max_facets = 4096, max_pairs = 4096;
  std::size_t max_device_bytes = 64u << 20, max_host_bytes = 16u << 20;
};
struct Forecast {
  std::size_t facets = 0, pairs = 0;
  std::size_t device_bytes = 0, owned_host_bytes = 0, startup_host_bytes = 0;
  std::size_t device_allocations = 0;
};
struct Preflight { Report report; Forecast forecast; };
}  // namespace tlfea::contact::self_contact_filters
