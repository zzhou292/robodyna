// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactActiveUseBinding.h"
#include "SelfContactPhysicalActivityTypes.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"

#include <memory>

namespace tlfea::contact {

// Fixed-capacity authority for one immutable self-contact selection and one
// actual physical publication. The owner, publication, binding and every
// participant must outlive this object. It allocates one activity arena at
// startup and performs no allocation in capture/discard/view operations.
class SelfContactPhysicalActivity {
 public:
  SelfContactPhysicalActivity() noexcept;
  ~SelfContactPhysicalActivity();
  SelfContactPhysicalActivity(const SelfContactPhysicalActivity&) = delete;
  SelfContactPhysicalActivity& operator=(
      const SelfContactPhysicalActivity&) = delete;
  SelfContactPhysicalActivity(SelfContactPhysicalActivity&&) = delete;
  SelfContactPhysicalActivity& operator=(
      SelfContactPhysicalActivity&&) = delete;

  static SelfContactPhysicalActivityPreflight Forecast(
      const SelfContactActiveUseBinding&,
      const tl::fea::ShellPhysicalBinding&,
      SelfContactPhysicalActivityLimits = {}) noexcept;

  // Calls both complete physical-source and accepted-activity-source
  // validators before retaining the exact handles and participant addresses.
  SelfContactPhysicalActivityReport Initialize(
      const SelfContactActiveUseBinding&,
      tl::fea::FENodalState&,
      tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,
      const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,
      SelfContactPhysicalActivityLimits = {}) noexcept;

  // Must follow successful accepted assembly by the actual physical
  // participants. No activity bytes are accepted from the caller. Failure
  // preserves the receipt output and activity arrays but invalidates any
  // previously live generation.
  SelfContactPhysicalActivityReport CaptureAccepted(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&,
      SelfContactAcceptedActivityReceipt*) noexcept;

  // Must follow successful participant diagnostics and PreparePhysical for
  // this exact owner/token/view. Any 0->1 selected-parent transition rejects.
  // Failure preserves output/base/current bytes and invalidates the attempt;
  // success supersedes the accepted receipt with a prepared generation.
  SelfContactPhysicalActivityReport CapturePrepared(
      tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,
      const tl::fea::ShellPhysicalDiagnostics&,
      const tl::fea::NodalPreparedView&,
      const SelfContactAcceptedActivityReceipt&,
      SelfContactPreparedActivityReceipt*) noexcept;

  // Invalidates the live attempt/receipts without modifying retained accepted
  // bytes or any participant/owner accepted state.
  void DiscardTrial() noexcept;

  bool initialized() const noexcept { return bool(state_); }
  SelfContactPhysicalActivityForecast forecast() const noexcept;
  SelfContactPhysicalActivityAllocationInfo allocations() const noexcept;

 private:
  std::shared_ptr<self_contact_physical_activity::State> state_;
};

}  // namespace tlfea::contact
