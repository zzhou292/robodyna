// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "mapped/State.h"
#include "../../../assembly/Type13NodeContributions.h"
#include "../../../solvers/NodalCinRuntime.h"
#include "../../../solvers/NodalTrialIdentity.h"

namespace tl::fea::type13::batch_detail {
BatchReport BuildStartup(const BatchConfig&, const Type13NodeContributions&,
                          util::HostArena&, const ArenaLayout&, Storage&,
                          BatchDiagnostics&);
BatchReport SourceForecast(const BatchConfig&, const Type13NodeContributions&,
    std::size_t private_bytes, BatchForecast&) noexcept;
BatchReport SourceGeometryPreflight(const BatchConfig&, const Type13NodeContributions&) noexcept;
BatchReport BuildSourceValues(const BatchConfig&, const Type13NodeContributions&,
    util::HostArena&, const ArenaLayout&, Storage&, BatchDiagnostics&);
BatchReport SourcePreflight(const BatchConfig&, const Type13NodeContributions&) noexcept;
bool SameDiagnostics(const BatchDiagnostics&, const BatchDiagnostics&) noexcept;
void LaunchAssembly(Storage*, unsigned slab, NodalAssemblyView,
                     NodalCinAssemblyView, bool initial, bool mapped = false);
void LaunchCandidate(Storage*, unsigned accepted, unsigned trial,
                      NodalPreparedView, BatchDiagnostics, std::size_t count);
void LaunchFailure(NodalAssemblyView);
} // namespace tl::fea::type13::batch_detail

namespace tl::fea::type13 {
struct Batch::Impl {
  Impl(const BatchConfig& c, const Type13NodeContributions& s)
      :config(c), source(s), accepted_stamp(c.owner) {}
  ~Impl();
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;
  BatchConfig config;
  Type13NodeContributions source;
  std::unique_ptr<mapped::State> physical;
  NodalStamp accepted_stamp;
  batch_detail::ArenaLayout layout;
  batch_detail::Storage* device = nullptr;
  batch_detail::Storage device_header;
  batch_detail::Control control;
  std::unique_ptr<Evaluation[]> staging;
  BatchDiagnostics accepted_diagnostics, candidate_diagnostics;
  NodalPreparedView candidate_view;
  cudaStream_t stream = nullptr;
  std::size_t host_bytes = 0;
  unsigned accepted_slab = 0;
  std::uint64_t assembled_epoch = UINT64_MAX;
  std::uint64_t assembled_attempt = 0, last_candidate_attempt = 0;
  bool usable = true, bound = false, pending = false;
  const ShellBatchPublication* publication_scope = nullptr;
  std::size_t Count() const noexcept { return source.model()->connection_count(); }
  unsigned TrialSlab() const noexcept { return 1 - accepted_slab; }
  void Discard() noexcept {
    pending = false;
    candidate_view = {};
    candidate_diagnostics = {};
  }
  BatchReport Upload(util::HostArena&, const batch_detail::Storage&);
  BatchReport Runtime(cudaError_t, const char*) noexcept;
  BatchReport PendingError() noexcept;
  BatchReport ReadControl();
  BatchReport ReadResults(unsigned slab);
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
};
} // namespace tl::fea::type13
