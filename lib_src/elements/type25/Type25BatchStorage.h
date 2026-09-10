// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25BatchArena.h"
#include "Type25BatchIdentity.h"
#include "Type25Model.h"
#include "../../assembly/NodalMassBinding.h"
#include <optional>
#include <utility>

namespace tl::fea::type25::batch_detail {
void LaunchAssembly(Storage*,const Slab*,NodalAssemblyView,bool initial);
void LaunchCandidate(Storage*,const Slab*,Slab*,NodalPreparedView,BatchDiagnostics,std::size_t element_count);
void LaunchFailure(NodalAssemblyView);
BatchReport BuildStartup(const BatchConfig&,const Model&,const NodalMassBinding&,util::HostArena&,
                        const ArenaLayout&,Storage& host_header,BatchDiagnostics&);
} // namespace tl::fea::type25::batch_detail

namespace tl::fea::type25 {
struct Batch::Impl {
  BatchConfig config;
  NodalStamp accepted_stamp;
  std::optional<Model> source;
  std::optional<NodalMassBinding> combined;
  const ShellBatchPublication* publication_scope=nullptr;
  batch_detail::Storage* storage=nullptr;
  batch_detail::Storage device_header;
  batch_detail::ArenaLayout layout;
  batch_detail::Slab* accepted=nullptr;
  batch_detail::Slab* trial=nullptr;
  batch_detail::Control control;
  std::unique_ptr<Evaluation[]> staging;
  BatchDiagnostics accepted_diagnostics,candidate_diagnostics;
  NodalPreparedView candidate_view;
  cudaStream_t stream=nullptr;
  std::size_t host_payload_bytes=0;
  std::uint64_t assembled_epoch=UINT64_MAX,assembled_attempt=0,last_candidate_attempt=0;
  bool usable=true,bound=false,pending=false;
  ~Impl();
  BatchReport Runtime(cudaError_t,const char*) noexcept;
  BatchReport PendingError() noexcept;
  BatchReport ReadControl();
  BatchReport ReadResults(const batch_detail::Slab*);
  bool OutputDisjoint(const void*,std::size_t) const noexcept;
  unsigned AcceptedSlabIndex() const noexcept { return accepted==&storage->slab[0]?0u:1u; }
  void Discard() noexcept { pending=false;candidate_view={};candidate_diagnostics={}; }
  void Publish(const NodalStamp& stamp) noexcept {
    std::swap(accepted,trial);accepted_stamp=stamp;accepted_diagnostics=candidate_diagnostics;
    accepted_diagnostics.phase=BatchPhase::Accepted;Discard();
  }
};
} // namespace tl::fea::type25
