// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalWallContactDevice.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/elements/ShellMappedFootprint.h"
#include "lib_src/constraints/NodalRigidAssemblyBinding.h"
#include "lib_src/solvers/NodalCinRuntime.h"

namespace tlfea::contact {
// Exact physical sources, never a caller mass/inverse/activity substitute.
// Immutable handles are retained; owner, publisher and participants must outlive
// calls. CIN ranges are borrowed only during initial authentication.
struct NodalWallMappedSource {
  const tl::fea::ShellPhysicalBinding* physical=nullptr;
  const tl::fea::NodalRigidAssemblyBinding* rigid=nullptr;
  tl::fea::NodalCinWitnessSource cin;
  tl::fea::ShellBatchPublication* publication=nullptr;
  tl::fea::ShellPhysicalParticipants participants;
  tl::fea::ShellPhysicalPublicationIdentity identity;
};
struct NodalWallMappedLimits {
  std::size_t max_startup_host_bytes=64u<<20;
  static constexpr NodalWallMappedLimits Vehicle() noexcept { return {8ull<<30}; }
};
struct NodalWallMappedDiagnostics {
  NodalWallDiagnostics contact;
  // Candidate contact potential uses the SAME accepted mask as drift work.
  // Removal at candidate coordinates is a separate nonnegative diagnostic.
  Q4CertifiedIntegral removed_potential;
  double current_response_rate_upper=0;
  std::size_t accepted_active_parents=0,proposed_active_parents=0;
  bool prepared_activity_available=false,valid=false;
  // Execution-route observation only; false for base/serial interval results.
  bool interval_tree_used=false;
};
class NodalWallMappedContact;
// Candidate-completion authority minted only after the concrete mapped wall
// has validated its actual prepared geometry, activity and interval result.
class NodalWallMappedTransactionReceipt {
 public:
  NodalWallMappedTransactionReceipt() noexcept = default;
  bool valid() const noexcept {
    return transaction_!=nullptr && owner_!=nullptr && wall_binding_id_!=0 &&
        attempt_!=0 && participation_.valid();
  }
  std::uint64_t wall_binding_id() const noexcept { return wall_binding_id_; }
  tl::fea::ShellPhysicalScratchReceiptRoster scratch_receipts()
      const noexcept {
    return valid()
        ? tl::fea::ShellPhysicalScratchReceiptRoster{&participation_,nullptr}
        : tl::fea::ShellPhysicalScratchReceiptRoster{};
  }

 private:
  friend class NodalWallMappedContact;
  const NodalWallMappedContact* transaction_=nullptr;
  const tl::fea::FENodalState* owner_=nullptr;
  std::uint64_t wall_binding_id_=0;
  std::uint64_t owner_id_=0,base_epoch_=0,attempt_=0;
  tl::fea::ShellPhysicalScratchParticipationReceipt participation_;
};
// Physical finite-wall scratch contributor. No accepted material/contact state,
// clock, owner commit or participant claim is added. Explicit zero damping,
// friction and offset; free world shell DOFs, actual PART/plain members and CIN
// masters are admitted. CIN secondary surface nodes remain unsupported. This
// first profile requires present rotations at surface nodes only; extra domain
// nodes may have absent rotations. Failed attempts require common discard.
// Response rate is frozen local screening, not global nonlinear stability.
class NodalWallMappedContact {
 public:
  NodalWallMappedContact();
  ~NodalWallMappedContact();
  NodalWallMappedContact(const NodalWallMappedContact&)=delete;
  NodalWallMappedContact& operator=(const NodalWallMappedContact&)=delete;
  static NodalWallDeviceReport Forecast(const NodalWallDeviceConfig&,const NodalWallWeights&,
      const NodalWallMappedSource&,tl::fea::ShellMappedFootprint&,
      NodalWallMappedLimits={}) noexcept;
  NodalWallDeviceReport Initialize(const NodalWallDeviceConfig&,PlanarWallView,
      const NodalWallWeights&,const NodalWallMappedSource&,tl::fea::FENodalState&,
      PlanarWallBox,NodalWallMappedLimits={});
  NodalWallDeviceReport AssembleAccepted(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&,NodalWallMappedDiagnostics*);
  NodalWallDeviceReport EvaluateCandidate(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&,
      NodalWallMappedDiagnostics*);
  NodalWallDeviceReport EvaluateCandidate(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&,
      NodalWallMappedDiagnostics*,NodalWallMappedTransactionReceipt*);
  // Available only after successful immutable wall initialization.  The
  // wall-binding identity is retained from config; callers supply no source ID.
  tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;
  // Failure-atomic exact-capacity records. row.valid and local timestep remain
  // unavailable for mapped nodes; no fake independent member response is stored.
  NodalWallDeviceReport CopyResults(const NodalWallMappedDiagnostics&,
      const NodalWallDeviceResultView&);
  void DiscardTrial() noexcept;
  tl::fea::NodalAllocationInfo allocations() const noexcept;
 private:
  void DiscardLocal() noexcept;
  NodalWallDeviceReport FailConfigured(
      NodalWallDeviceReport) noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
  tl::fea::ShellPhysicalScratchParticipation participation_;
};
} // namespace tlfea::contact
