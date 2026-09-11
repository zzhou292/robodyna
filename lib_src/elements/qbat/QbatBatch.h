// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatBatchTypes.h"
#include "../ShellFormulationScope.h"
#include <memory>

namespace tl::fea { class ShellBatchPublication; }
namespace tl::fea::qbat {

// One complete native QBAT family from a prepared formulation inventory.
// The owner supplies all nodal state, clock, stream and actual transaction views.
// One accepted selector controls both the four-point history and force cache.
// Initial reference/rest or common translation binds a real zero cache without
// a dt=0 force call. No independent commit, mass removal or per-step allocation.
class Batch {
 public:
  Batch();
  ~Batch();
  Batch(const Batch&)=delete;
  Batch& operator=(const Batch&)=delete;
  BatchReport InitializeFormulations(const BatchConfig&,const ShellFormulationScope&);
  BatchReport AssembleAccepted(FENodalState&,const NodalAssemblyView&);
  BatchReport EvaluateCandidate(FENodalState&,const NodalTrialToken&,
      const NodalPreparedView&,BatchDiagnostics*);
  // Output is staged and completely validated before publication. Rows use the
  // immutable complete family's source order, never a PID-filtered subset.
  // Named fields are values; object padding is not an equality/hash contract.
  BatchReport CopyAcceptedResults(const NodalStamp&,BatchResult*,std::size_t capacity,BatchDiagnostics*);
  BatchReport CopyPreparedResults(const BatchDiagnostics&,BatchResult*,std::size_t capacity);
  BatchReport CopyAcceptedDiagnostics(const NodalStamp&,BatchDiagnostics*) const noexcept;
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
  std::size_t host_bytes() const noexcept;
 private:
  friend class ::tl::fea::ShellBatchPublication;
  BatchReport PreflightAttach(FENodalState&,const ShellFormulationScope&,
      std::uint64_t configuration,std::uint64_t qualification,const ShellBatchStartup&,
      const ShellBatchPublication*) const noexcept;
  BatchReport PreflightPublication(FENodalState&,const NodalTrialToken&,const NodalPreparedView&,
      const BatchDiagnostics&,const ShellBatchPublication*) const noexcept;
  void AttachPublication(const ShellBatchPublication*) noexcept;
  void ReleasePublication(const ShellBatchPublication*) noexcept;
  void Publish(const NodalStamp&) noexcept;
  void Poison() noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
namespace batch_detail {
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
}
} // namespace tl::fea::qbat
