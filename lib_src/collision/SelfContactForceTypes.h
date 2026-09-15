// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedTriangleFeatureTypes.h"
#include "SelfContactActiveUseTypes.h"
#include "penalty_pair/Types.h"
#include "lib_src/elements/ShellBatchStartup.h"
#include "lib_src/solvers/FENodalState.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace tlfea::contact {

enum class SelfContactForceStatus : std::uint8_t {
  Ok,
  AlreadyInitialized,
  NotInitialized,
  InvalidInput,
  ResourceLimit,
  IdentityMismatch,
  DuplicateEvent,
  StaleAttempt,
  EventFailure,
  AssemblyFailure,
  OwnerFailure,
  DeviceFailure,
};

struct SelfContactForceReport {
  SelfContactForceStatus status = SelfContactForceStatus::Ok;
  std::size_t event = SIZE_MAX;
  std::uint64_t source_order = UINT64_MAX;
  std::uint32_t node = UINT32_MAX;
  SurfacePenaltyStatus pair_status = SurfacePenaltyStatus::Ok;
  tl::fea::NodalStatus owner_status = tl::fea::NodalStatus::Ok;
  const char* message = "OK";
};

struct SelfContactForceConfig {
  // Exact fresh physical owner declaration. The startup value is consumed by
  // the existing complete ledger/CIN authentication; neither is a second clock.
  tl::fea::NodalStamp owner;
  tl::fea::ShellBatchStartup startup;
  double stiffness_per_area_n_m3 = 0;
  std::size_t event_capacity = 0;
  std::uint64_t configuration_id = 0;
  std::uint64_t qualification_id = 0;
};

struct SelfContactForceLimits {
  std::size_t max_events = 4096;
  std::size_t max_nodes = 2048;
  std::size_t max_host_bytes = 64u << 20;
  std::size_t max_device_bytes = 64u << 20;
  std::size_t max_startup_host_bytes = 256u << 20;
  static constexpr SelfContactForceLimits Vehicle() noexcept {
    return {1u << 20, 524288, 4ull << 30, 4ull << 30, 24ull << 30};
  }
};

struct SelfContactForceForecast {
  std::size_t event_capacity = 0;
  std::size_t incidence_capacity = 0;
  std::size_t touched_node_capacity = 0;
  std::size_t staged_channel_values = 0;
  std::size_t host_arena_bytes = 0;
  std::size_t device_bytes = 0;
  std::size_t device_allocations = 0;
  std::size_t retained_active_use_bytes = 0;
  std::size_t owned_host_bytes = 0;
  std::size_t startup_scratch_bytes = 0;
  std::size_t startup_host_bytes = 0;
};

struct SelfContactForcePreflight {
  SelfContactForceReport report;
  SelfContactForceForecast forecast;
};

// One already-discovered and active-use-resolved VF or symmetric EE event.
// Geometry discovery and a same-attempt candidate receipt intentionally remain
// outside this accepted-state scratch contributor.
struct SelfContactForceEventIdentity {
  FixedTriangleFeatureKey feature;
  std::uint32_t parent[2]{UINT32_MAX, UINT32_MAX};
};

static_assert(std::is_trivially_copyable_v<SelfContactForceEventIdentity>);

struct SelfContactForceEvent {
  FixedTriangleFeatureKey feature;
  std::uint64_t source_order = UINT64_MAX;
  // Exact active-use ordinals used to regenerate the complete classification.
  std::uint32_t vertex_use = UINT32_MAX;
  std::uint32_t facet_use = UINT32_MAX;
  std::uint32_t edge_use[2]{UINT32_MAX, UINT32_MAX};
  WeightedSurfacePoint endpoints[2];
  SelfContactPairClassification classification;
};

struct SelfContactForceEventView {
  const SelfContactForceEvent* data = nullptr;
  std::size_t count = 0;
};

struct SelfContactForceIncidence {
  std::uint32_t node = UINT32_MAX;
  std::uint32_t event = UINT32_MAX;
};

struct SelfContactForceNodeIncidence {
  std::uint32_t node = UINT32_MAX;
  std::uint32_t offset = 0;
  std::uint32_t count = 0;
};

struct SelfContactForceIncidenceSummary {
  std::size_t incidences = 0;
  std::size_t touched_nodes = 0;
};

struct SelfContactForceDiagnostics {
  std::size_t event_count = 0;
  std::size_t vertex_face_event_count = 0;
  std::size_t boundary_vertex_edge_event_count = 0;
  std::size_t edge_edge_event_count = 0;
  std::size_t active_count = 0;
  Vec3 endpoint_a_resultant_n;
  Vec3 endpoint_b_resultant_n;
  Vec3 equal_opposite_residual_n;
  Vec3 global_moment_n_m;
  double potential_j = 0;
  double maximum_force_norm_n = 0;
  double maximum_sti_diagonal_n_m = 0;
  double maximum_represented_stiffness_n_m = 0;
  std::uint64_t owner_id = 0;
  std::uint64_t base_epoch = 0;
  std::uint64_t attempt = 0;
  std::uint64_t configuration_id = 0;
  std::uint64_t qualification_id = 0;
  std::uint64_t first_source_order = UINT64_MAX;
  std::uint64_t last_source_order = UINT64_MAX;
  const void* active_use_identity = nullptr;
  tl::fea::NodalTemporalScheme temporal_scheme =
      tl::fea::NodalTemporalScheme::VelocityFirst;
  tl::fea::NodalVelocityPhase velocity_phase =
      tl::fea::NodalVelocityPhase::Collocated;
  double position_time = 0;
  double velocity_time = 0;
  bool valid = false;
};

class SelfContactForceAssembly;

class SelfContactForceAssemblyReceipt {
 public:
  SelfContactForceAssemblyReceipt() noexcept = default;
  // Diagnostic completeness only. Authority consumers must ask the live
  // assembler to authenticate its nonrepeating startup identity; this value
  // alone cannot authorize work, including after assembler destruction.
  bool prepared() const noexcept {
    return assembler_ != nullptr && assembler_identity_ != 0 &&
        diagnostics_.valid;
  }
  const SelfContactForceDiagnostics& diagnostics() const noexcept {
    return diagnostics_;
  }

 private:
  friend class SelfContactForceAssembly;
  const SelfContactForceAssembly* assembler_ = nullptr;
  std::uint64_t assembler_identity_ = 0;
  SelfContactForceDiagnostics diagnostics_;
};

}  // namespace tlfea::contact
