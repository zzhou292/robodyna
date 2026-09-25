// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/assembly/DeviceTypes.h"
#include <memory>
struct CUstream_st;
namespace tlfea::contact::radioss_type25::assembly {
class DeviceIncidenceBuilder;
class DeviceIncidenceView {
 public:
  Incidence incidence() const noexcept { return incidence_; }
  IncidenceStamp stamp() const noexcept { return stamp_; }
 private:
  friend class DeviceIncidenceBuilder;
  std::uint64_t owner_ = 0, sequence_ = 0;
  Incidence incidence_;
  IncidenceStamp stamp_;
};
// Bounded numerical workspace, not a physical state/history owner. Stage uses
// the explicit startup stream, drains launched work before return, and copies only one
// integer diagnostic word to the host. No per-stage allocation or map download.
// Every Stage attempt or Discard expires the previous view. The coordinator
// keeps accepted and trial instances separate if it retains both incidences.
// Borrowed inputs remain immutable/readable until Stage returns. Afterwards
// the coordinator authenticates matching connectivity/cohort/packet generations
// before using the returned CSR with GatherNode on the same ordered stream.
// Host admission failures launch no work and do not wait for unrelated caller
// operations that were already queued on that stream.
class DeviceIncidenceBuilder {
 public:
  DeviceIncidenceBuilder();
  ~DeviceIncidenceBuilder();
  DeviceIncidenceBuilder(const DeviceIncidenceBuilder&) = delete;
  DeviceIncidenceBuilder& operator=(const DeviceIncidenceBuilder&) = delete;
  // Queries CUB/CUDA startup scratch; not a GPU-free parser operation.
  static IncidenceStatus Preflight(IncidenceLimits, IncidenceForecast&) noexcept;
  IncidenceStatus Initialize(IncidenceLimits, CUstream_st*) noexcept;
  IncidenceStatus Stage(const DeviceConnectivity&) noexcept;
  DeviceIncidenceView view() const noexcept;
  bool IsCurrent(const DeviceIncidenceView&) const noexcept;
  void Discard() noexcept;
  IncidenceReport last_report() const noexcept;
  IncidenceForecast allocations() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact::radioss_type25::assembly
