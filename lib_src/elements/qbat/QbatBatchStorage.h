// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatBatch.h"
#include "QbatBatchArena.h"
#include "QbatBatchIdentity.h"
#include "../../assembly/NodalMassBinding.h"
#include <optional>
#include <utility>

namespace tl::fea::qbat::batch_detail {
void LaunchAssembly(Storage*,const Slab*,NodalAssemblyView,bool initial);
void LaunchCandidate(Storage*,const Slab*,Slab*,NodalPreparedView,BatchDiagnostics,std::size_t count);
void LaunchFailure(NodalAssemblyView);
} // namespace tl::fea::qbat::batch_detail

namespace tl::fea::qbat {
struct Batch::Impl {
  BatchConfig config;
  NodalStamp accepted_stamp;
  std::optional<ShellBatchBinding> binding;
  std::optional<ShellBatchFailureBinding> failure;
  std::optional<NodalMassBinding> combined;
  const ShellBatchPublication* publication_scope=nullptr;
  batch_detail::Storage* storage=nullptr;
  batch_detail::Storage device_header;
  batch_detail::Layout layout;
  batch_detail::Slab* accepted=nullptr;
  batch_detail::Slab* trial=nullptr;
  batch_detail::Control control;
  std::unique_ptr<BatchResult[]> staging;
  BatchDiagnostics accepted_diagnostics,candidate_diagnostics;
  NodalPreparedView candidate_view;
  NodalAssemblyView initial_sources;
  cudaStream_t stream=nullptr;
  std::size_t host_payload_bytes=0;
  std::uint64_t assembled_epoch=UINT64_MAX,assembled_attempt=0,last_candidate_attempt=0;
  bool usable=true,bound=false,pending=false;
  ~Impl();
  BatchReport Runtime(cudaError_t,const char*) noexcept;
  BatchReport PendingError() noexcept;
  BatchReport ReadControl();
  BatchReport ReadResults(const batch_detail::Slab*,const BatchDiagnostics&);
  bool OutputDisjoint(const void*,std::size_t) const noexcept;
  ShellFormulationScope Scope() const noexcept {
    return {binding?&*binding:nullptr,failure?failure->catalog():nullptr,
        failure?&*failure:nullptr,combined?&*combined:nullptr};
  }
  void Discard() noexcept {
    pending=false;
    candidate_view={};
    candidate_diagnostics={};
  }
  void Publish(const NodalStamp& stamp) noexcept {
    std::swap(accepted,trial);
    accepted_stamp=stamp;
    accepted_diagnostics=candidate_diagnostics;
    accepted_diagnostics.phase=BatchPhase::Accepted;
    Discard();
  }
};
} // namespace tl::fea::qbat
