// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "physical_activity/Types.h"
#include "../../assembly/ShellPhysicalBinding.h"
namespace tl::fea {
// One common-owner observer, shared by contact interfaces. It borrows the
// actual publication and participants for its complete lifetime, allocates
// only at initialization and never changes their physical state or clock.
class PhysicalActivitySnapshot {
 public:
  PhysicalActivitySnapshot() noexcept;
  ~PhysicalActivitySnapshot();
  PhysicalActivitySnapshot(const PhysicalActivitySnapshot&) = delete;
  PhysicalActivitySnapshot& operator=(const PhysicalActivitySnapshot&) = delete;
  static PhysicalActivityReport Preflight(const ShellPhysicalBinding&,
      PhysicalActivityLimits, PhysicalActivityForecast&) noexcept;
  PhysicalActivityReport Initialize(FENodalState&, ShellBatchPublication&,
      const ShellPhysicalBinding&, const ShellPhysicalParticipants&,
      const ShellPhysicalPublicationIdentity&, PhysicalActivityLimits = {}) noexcept;
  PhysicalActivityReport CaptureAccepted(FENodalState&, const NodalTrialToken&,
      const NodalAssemblyView&, PhysicalAcceptedActivityReceipt*) noexcept;
  PhysicalActivityReport CapturePrepared(FENodalState&, const NodalTrialToken&,
      const ShellPhysicalDiagnostics&, const NodalPreparedView&,
      const PhysicalAcceptedActivityReceipt&, PhysicalPreparedActivityReceipt*) noexcept;
  // These fresh source/owner/phase checks cannot be replaced with valid().
  // Repeated borrows share the completed capture without repeating GPU work.
  PhysicalActivityReport BorrowAccepted(FENodalState&, const NodalTrialToken&,
      const NodalAssemblyView&, const PhysicalAcceptedActivityReceipt&,
      PhysicalActivityDeviceView*) const noexcept;
  PhysicalActivityReport BorrowPrepared(FENodalState&, const NodalTrialToken&,
      const ShellPhysicalDiagnostics&, const NodalPreparedView&,
      const PhysicalPreparedActivityReceipt&, PhysicalActivityDeviceView*) const noexcept;
  // SourcePlan may name only the actual attached massless-joint model.
  // Null is explicit and succeeds only when the common roster has no TYPE45.
  PhysicalActivityReport ValidateType45Source(const type45::Model*) const noexcept;
  void DiscardTrial() noexcept;
  PhysicalActivityForecast allocations() const noexcept;
 private:
  std::shared_ptr<physical_activity::State> state_;
};
} // namespace tl::fea
