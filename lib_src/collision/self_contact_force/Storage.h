// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactForceAssembly.h"
#include "lib_src/elements/ShellPhysicalOwner.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact_force {

struct EventStatus {
  SelfContactForceStatus status = SelfContactForceStatus::Ok;
  SurfacePenaltyStatus pair_status = SurfacePenaltyStatus::Ok;
};

struct NodeStatus {
  SelfContactForceStatus status = SelfContactForceStatus::Ok;
};

struct Control {
  SelfContactForceStatus status = SelfContactForceStatus::Ok;
  SurfacePenaltyStatus pair_status = SurfacePenaltyStatus::Ok;
  std::uint64_t source_order = UINT64_MAX;
  std::uint32_t event = UINT32_MAX;
  std::uint32_t node = UINT32_MAX;
};

struct Layout {
  tl::util::ArenaRegion events;
  tl::util::ArenaRegion packets;
  tl::util::ArenaRegion incidences;
  tl::util::ArenaRegion nodes;
  tl::util::ArenaRegion event_status;
  tl::util::ArenaRegion node_status;
  tl::util::ArenaRegion staged_channels;
  tl::util::ArenaRegion staged_sti;
  tl::util::ArenaRegion control;
  tl::util::ArenaRegion diagnostics;
  std::size_t bytes = 0;
};

struct Buffers {
  SelfContactForceEvent* events = nullptr;
  SurfacePenaltyPacket* packets = nullptr;
  SelfContactForceIncidence* incidences = nullptr;
  SelfContactForceNodeIncidence* nodes = nullptr;
  EventStatus* event_status = nullptr;
  NodeStatus* node_status = nullptr;
  double* staged_channels = nullptr;
  double* staged_sti = nullptr;
  Control* control = nullptr;
  SelfContactForceDiagnostics* diagnostics = nullptr;
};

bool MakeLayout(std::size_t events, std::size_t nodes,
                std::size_t max_bytes, Layout&) noexcept;
Buffers Bind(void*, const Layout&) noexcept;
SelfContactForcePreflight Preflight(
    const SelfContactForceConfig&,
    const SelfContactActiveUseBinding&,
    SelfContactForceLimits,
    std::size_t owner_bytes,
    Layout* = nullptr,
    tl::fea::shell_physical_owner::ProofLayout* = nullptr) noexcept;

bool SamePoint(const WeightedSurfacePoint&,
               const WeightedSurfacePoint&) noexcept;
bool SameSupport(const SelfContactSupportClassification&,
                 const SelfContactSupportClassification&) noexcept;
bool SameCertificate(Q4CertifiedIntegral, Q4CertifiedIntegral) noexcept;
SelfContactForceReport ValidateEvent(
    const SelfContactActiveUseBinding&,
    const SelfContactForceEvent&,
    SelfContactActivityView,
    std::size_t canonical_event) noexcept;

}  // namespace tlfea::contact::self_contact_force

namespace tlfea::contact {

struct SelfContactForceAssembly::Impl {
  explicit Impl(const SelfContactActiveUseBinding& source)
      : binding(source) {}
  ~Impl();

  SelfContactActiveUseBinding binding;
  tl::fea::FENodalState* owner = nullptr;
  SelfContactForceConfig config;
  SelfContactForceForecast storage_forecast;
  self_contact_force::Layout layout;
  tl::util::HostArena host;
  self_contact_force::Buffers local;
  self_contact_force::Buffers remote;
  void* device = nullptr;
  self_contact_force::Control control;
  cudaStream_t stream = nullptr;
  std::uint64_t assembler_identity = 0;
  std::uint64_t last_attempt = 0, last_base_epoch = 0;
  bool usable = true;

  SelfContactForceReport Check(cudaError_t) noexcept;
  SelfContactForceReport ReadControlAndDiagnostics(
      SelfContactForceDiagnostics&) noexcept;
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
};

}  // namespace tlfea::contact
