// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactTransactionTypes.h"
#include "SelfContactTransactionDiagnostics.h"

#include <memory>

namespace tlfea::contact {

namespace self_contact_transaction {
class QualificationAccess;
struct CandidateFailureObserver;
}

// Fixed-capacity runtime composition for one self-contact source. It privately
// owns broadphase/readback/expansion, accepted and candidate feature discovery,
// current regularity, crossing, exact event certificates, the force assembler,
// and exactly one physical scratch issuer. Snapshots are attempt scratch, not
// accepted state or an independent clock.
//
// CUDA execution order is inherently nondeterministic. This composition does
// not infer determinism from launch order or timing: broadphase publishes
// sorted integer pair keys, CPU workers have one writer per canonical pair,
// force/STI uses one writer per canonical node and fixed event-order folds,
// and all device phases use the authenticated owner stream. Under identical
// binary64 inputs and build flags, those algorithmic rules define bitwise
// output determinism; elapsed time is never part of acceptance.
class SelfContactTransaction {
 public:
  SelfContactTransaction() noexcept;
  ~SelfContactTransaction();
  SelfContactTransaction(const SelfContactTransaction&) = delete;
  SelfContactTransaction& operator=(const SelfContactTransaction&) = delete;
  SelfContactTransaction(SelfContactTransaction&&) = delete;
  SelfContactTransaction& operator=(SelfContactTransaction&&) = delete;

  static SelfContactTransactionPreflight Forecast(
      const SelfContactTransactionConfig&,
      const SelfContactActiveUseBinding&,
      const tl::fea::ShellPhysicalPublicationIdentity&,
      SelfContactTransactionLimits = {}) noexcept;

  SelfContactTransactionReport Initialize(
      const SelfContactTransactionConfig&,
      const SelfContactActiveUseBinding&,
      tl::fea::FENodalState&,
      tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,
      const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,
      cudaStream_t owner_stream,
      SelfContactTransactionLimits = {});

  // The sole externally visible issuer capability. The publisher still
  // authenticates and binds it before interval 1.
  tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;

  SelfContactTransactionReport AssembleAccepted(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&,
      SelfContactAcceptedAssemblyReceipt*);

  // Requires and authenticates the exact common diagnostics produced by
  // PreparePhysical. Actual accepted/prepared owner coordinates and physical
  // activity are captured before regularity, discovery, and interval crossing.
  SelfContactTransactionReport SealCandidate(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::ShellPhysicalDiagnostics&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      SelfContactTransactionReceipt*);

  // Borrowed policy publication for the currently sealed candidate. It is
  // revoked by discard, a new accepted assembly, failure, or destruction.
  // Large streams may publish only policy_summary(); this view is complete
  // only when the caller reserved the entire detailed outcome census.
  SelfContactCandidatePolicyView policy_outcomes() const noexcept;
  SelfContactCandidatePolicySummary policy_summary() const noexcept;

  // Actual immutable initialization route; does not count or authorize queries.
  SelfContactFacetFilterInitialization facet_filter_initialization() const noexcept;

  // Optional value-only host diagnostic snapshots; no publication authority.
  SelfContactTransactionDiagnostics diagnostics() const noexcept;

  // Contact-local composition. Common transaction abandonment still calls the
  // owner and ShellBatchPublication discard operations.
  void DiscardTrial() noexcept;

  bool initialized() const noexcept { return bool(impl_); }
  SelfContactTransactionForecast forecast() const noexcept;
  SelfContactTransactionAllocationInfo allocations() const noexcept;

 private:
  friend class self_contact_transaction::QualificationAccess;
  SelfContactTransactionReport SealCandidateImpl(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::ShellPhysicalDiagnostics&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      SelfContactTransactionReceipt*,
      const self_contact_transaction::CandidateFailureObserver*);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tlfea::contact
