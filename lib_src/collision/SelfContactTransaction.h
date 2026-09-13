// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactTransactionTypes.h"

#include <memory>

namespace tlfea::contact {

// Fixed-capacity runtime composition for one self-contact source. It privately
// owns the force assembler and exactly one physical scratch issuer. It owns no
// accepted state, contact history, timestep policy, or independent clock.
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
      SelfContactTransactionLimits = {});

  // The sole externally visible issuer capability. The publisher still
  // authenticates and binds it before interval 1.
  tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;

  SelfContactTransactionReport AssembleAccepted(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&,
      SelfContactActivityView,
      SelfContactForceEventView,
      SelfContactAcceptedAssemblyReceipt*);

  // Requires the common physical candidate to have been prepared already.
  // Actual accepted/prepared owner coordinates are copied into fixed private
  // storage, then regularity, discovery, and interval crossing are rerun.
  SelfContactTransactionReport SealCandidate(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedAssemblyReceipt&,
      const SelfContactCandidateEvidence&,
      SelfContactTransactionReceipt*);

  // Contact-local composition. Common transaction abandonment still calls the
  // owner and ShellBatchPublication discard operations.
  void DiscardTrial() noexcept;

  bool initialized() const noexcept { return bool(impl_); }
  SelfContactTransactionForecast forecast() const noexcept;
  tl::fea::NodalAllocationInfo allocations() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tlfea::contact
