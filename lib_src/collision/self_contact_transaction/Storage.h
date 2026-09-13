// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactTransaction.h"
#include "../fixed_triangle_features/Geometry.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_transaction {

struct AcceptedEventIdentity {
  FixedTriangleFeatureKey feature;
  std::uint64_t source_order = UINT64_MAX;
};

struct Layout {
  tl::util::ArenaRegion accepted_positions;
  tl::util::ArenaRegion accepted_velocities;
  tl::util::ArenaRegion prepared_positions;
  tl::util::ArenaRegion prepared_velocities;
  tl::util::ArenaRegion activity;
  tl::util::ArenaRegion current_triangles;
  tl::util::ArenaRegion represented_paths;
  tl::util::ArenaRegion represented_pairs;
  tl::util::ArenaRegion canonical_pairs;
  tl::util::ArenaRegion accepted_events;
  std::size_t bytes = 0;
};

struct Buffers {
  double* accepted_positions = nullptr;
  double* accepted_velocities = nullptr;
  double* prepared_positions = nullptr;
  double* prepared_velocities = nullptr;
  std::uint8_t* activity = nullptr;
  CurrentFixedTriangle* current_triangles = nullptr;
  RepresentedTrianglePath* represented_paths = nullptr;
  RepresentedTrianglePair* represented_pairs = nullptr;
  RepresentedIntervalPairKey* canonical_pairs = nullptr;
  AcceptedEventIdentity* accepted_events = nullptr;
};

bool MakeLayout(std::size_t nodes, std::size_t parents,
                std::size_t triangle_capacity, std::size_t pair_capacity,
                std::size_t event_capacity, std::size_t max_bytes,
                Layout&) noexcept;
Buffers Bind(void*, const Layout&) noexcept;

int Compare(const RepresentedTrianglePathKey&,
            const RepresentedTrianglePathKey&) noexcept;
int Compare(const RepresentedIntervalPairKey&,
            const RepresentedIntervalPairKey&) noexcept;
bool Same(const FixedTriangleKey&, const FixedTriangleKey&) noexcept;
bool Same(const FixedTriangleIntersection&,
          const FixedTriangleIntersection&) noexcept;
bool Same(const FixedTriangleFeatureKey&,
          const FixedTriangleFeatureKey&) noexcept;

struct CandidateValidationInput {
  const RepresentedIntervalPairKey* canonical_pairs = nullptr;
  std::size_t pair_count = 0;
  FixedTriangleFeatureView features;
  FixedTriangleIntersectionView intersections;
  RepresentedIntervalResultView crossings;
  const SelfContactCrossingDecision* decisions = nullptr;
  std::size_t decision_count = 0;
  const AcceptedEventIdentity* accepted_events = nullptr;
  std::size_t accepted_event_count = 0;
};

SelfContactTransactionReport ValidateCandidatePublications(
    const CandidateValidationInput&) noexcept;

}  // namespace tlfea::contact::self_contact_transaction

namespace tlfea::contact {

struct SelfContactTransaction::Impl {
  enum class Phase : std::uint8_t {
    Idle,
    AssemblyRecorded,
    CandidateSealed,
  };

  explicit Impl(const SelfContactActiveUseBinding& source)
      : active_use(source) {}

  SelfContactActiveUseBinding active_use;
  tl::fea::FENodalState* owner = nullptr;
  tl::fea::ShellBatchPublication* publication = nullptr;
  SelfContactTransactionConfig config;
  SelfContactTransactionForecast storage_forecast;
  self_contact_transaction::Layout layout;
  tl::util::HostArena arena;
  self_contact_transaction::Buffers buffers;
  SelfContactForceAssembly force;
  tl::fea::ShellPhysicalScratchParticipation participation;
  const std::uint8_t* activity_base_identity = nullptr;
  const std::uint8_t* activity_current_identity = nullptr;
  std::size_t accepted_event_count = 0;
  std::uint64_t owner_id = 0;
  std::uint64_t base_epoch = 0;
  std::uint64_t attempt = 0;
  cudaStream_t stream = nullptr;
  Phase phase = Phase::Idle;

  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  void DiscardLocal() noexcept;
  SelfContactTransactionReport Fail(
      SelfContactTransactionReport) noexcept;
};

}  // namespace tlfea::contact
