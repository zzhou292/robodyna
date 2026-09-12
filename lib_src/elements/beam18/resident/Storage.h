// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "../../../solvers/NodalTrialIdentity.h"

namespace tl::fea::beam18::batch_detail {
bool SameDiagnostics(const BatchDiagnostics&, const BatchDiagnostics&) noexcept;
bool SameConfig(const BatchConfig&, const BatchConfig&) noexcept;
void LaunchInitialize(Storage*, cudaStream_t);
void LaunchCandidate(Storage*, unsigned accepted, unsigned trial,
    NodalPreparedView, BatchDiagnostics);
void LaunchAssembly(Storage*, unsigned accepted, NodalAssemblyView, NodalCinAssemblyView);
} // namespace tl::fea::beam18::batch_detail
namespace tl::fea::beam18 {
struct Batch::Impl {
  Impl(const BatchConfig& c, const Model& m) : config(c), model(m), accepted_stamp(c.owner) {}
  ~Impl();
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;
  BatchConfig config;
  Model model;
  NodalStamp accepted_stamp;
  batch_detail::ArenaLayout layout;
  batch_detail::Storage* device = nullptr;
  batch_detail::Storage device_header;
  batch_detail::Control control;
  util::HostArena staging;
  BatchDiagnostics accepted_diagnostics, candidate_diagnostics;
  NodalPreparedView candidate_view;
  cudaStream_t stream = nullptr;
  std::size_t host_bytes = 0;
  unsigned accepted_slab = 0;
  std::uint64_t assembled_epoch = UINT64_MAX;
  std::uint64_t assembled_attempt = 0, last_candidate_attempt = 0;
  bool usable = true, bound = false, pending = false;
  const ShellBatchPublication* publication_scope = nullptr;
  const ShellBatchPublication* preflight_claimant = nullptr;
  unsigned TrialSlab() const noexcept { return 1 - accepted_slab; }
  void Discard() noexcept {
    pending = false;
    candidate_view = {};
    candidate_diagnostics = {};
  }
  BatchReport Runtime(cudaError_t, const char*) noexcept;
  BatchReport PendingError() noexcept;
  BatchReport ReadControl();
  BatchReport ReadResults(unsigned slab, const BatchDiagnostics&);
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  bool OutputBuffers(ResultBuffer, const void*, std::size_t,
      const void*, std::size_t) const noexcept;
  void PublishResults(ResultBuffer) const noexcept;
};
} // namespace tl::fea::beam18
