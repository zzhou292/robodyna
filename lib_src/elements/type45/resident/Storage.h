// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
#include "../../../solvers/NodalTrialIdentity.h"

namespace tl::fea::type45 {
struct Batch::Impl {
  Impl(const BatchConfig& c,const Model& m):config(c),model(m),accepted_stamp(c.owner) {}
  ~Impl();
  Impl(const Impl&)=delete;
  Impl& operator=(const Impl&)=delete;
  BatchConfig config;
  Model model;
  NodalStamp accepted_stamp;
  resident_detail::ArenaLayout layout;
  resident_detail::Storage* device=nullptr;
  resident_detail::Storage device_header;
  resident_detail::Control control;
  util::HostArena staging;
  BatchDiagnostics accepted_diagnostics,candidate_diagnostics;
  NodalPreparedView candidate_view;
  cudaStream_t stream=nullptr;
  std::size_t host_bytes=0;
  unsigned accepted_slab=0;
  std::uint64_t assembled_epoch=UINT64_MAX,assembled_attempt=0,last_candidate_attempt=0;
  std::uint64_t cin_qualification_id=0;
  bool usable=true,bound=false,pending=false;
  const ShellBatchPublication* publication_scope=nullptr;
  const ShellBatchPublication* preflight_claimant=nullptr;
  // Borrowed only while claimed; the coordinator retains the authority. Cleared
  // by ReleasePublication, before the coordinator destroys its backing.
  const ShellPhysicalBinding* physical_scope=nullptr;
  unsigned TrialSlab() const noexcept {return 1-accepted_slab;}
  void Discard() noexcept {pending=false;candidate_view={};candidate_diagnostics={};}
  BatchReport Runtime(cudaError_t,const char*) noexcept;
  BatchReport PendingError() noexcept;
  BatchReport ReadControl();
  BatchReport ReadResults(unsigned,const BatchDiagnostics&);
  BatchReport PrepareContexts(FENodalState&,const NodalTrialToken&,const NodalPreparedView&);
  bool OutputDisjoint(const void*,std::size_t) const noexcept;
  bool OutputBuffer(ResultBuffer,const void*,std::size_t,const void*,std::size_t) const noexcept;
  void PublishResults(ResultBuffer) const noexcept;
};
} // namespace tl::fea::type45
