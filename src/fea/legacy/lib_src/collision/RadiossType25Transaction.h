// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/runtime/Types.h"
#include "RadiossType25InitialState.h"
#include "../elements/ShellBatchPublication.h"
#include "../elements/publication/NativeContactPublicationState.h"
#include <memory>
namespace tlfea::contact::radioss_type25 {
namespace runtime_qualification {class Access;}
struct TransactionGroupReport {
  TransactionReport report;
  std::size_t interface_index=SIZE_MAX;
};
struct AcceptedContactBuffer {
  NativeGeometryHistory* rows=nullptr;
  int* initial_contact_flags=nullptr;
  std::size_t row_capacity=0;
};
struct GeneralTransactionForecast {
  TransactionForecast transaction;
  initial_source::Forecast initializer;
  // Max of full runtime and primary runtime arena plus one producer peak.
  // The seed retires before auxiliary runtime allocation. Host peak includes
  // source staging and immutable PreparedSource retained through both phases.
  std::size_t peak_device_bytes=0,peak_host_bytes=0;
};
struct TransactionInitializationDiagnostics {
  bool available=false;
  initial_source::SeedIdentity identity;
  initial_source::Diagnostics values;
};
// Concrete explicitly selected fixed/moving-main GPU contact participant. FENodalState remains the sole
// clock and state owner. This stable-address object is noncopyable/nonmovable.
// Serialize every call/read/destruction with the owner and common publisher;
// owner and its borrowed stream must outlive destruction. The publisher may
// destruct first: its typed issuer detaches and all later calls reject before
// dereferencing the former publisher. All staging calls drain.
// Only ShellBatchPublication::CommitPhysical can publish accepted selectors.
class Transaction {
 public:
  Transaction();~Transaction();
  // Exact shared source/layout forecast before any owner or transaction device
  // allocation. Host source staging and CUDA scratch-size queries are bounded
  // by limits. Success does not authenticate a live publisher or accepted state.
  // Failure leaves output unchanged. Initialize repeats this same plan.
  static TransactionReport Preflight(const TransactionConfig&,const FixedMainSource&,
      const tl::fea::ShellPhysicalBinding&,TransactionForecast&,TransactionLimits={}) noexcept;
  static TransactionReport Preflight(const TransactionConfig&,const MovingMainSource&,
      const tl::fea::ShellPhysicalBinding&,TransactionForecast&,TransactionLimits={}) noexcept;
  static TransactionReport Preflight(const TransactionConfig&,const MixedMovingMainSource&,
      const tl::fea::ShellPhysicalBinding&,TransactionForecast&,TransactionLimits={}) noexcept;

  // Genuine Starter source preparation, including final geometric/TYPE2 CSR.
  // PreparedSource identity includes explicit native unit/domain/source/map
  // coherence and a source-proved fresh Engine search at time0. No arbitrary
  // caller history or ready-normal substitution enters this overload.
  static TransactionReport GeneralPreflight(const TransactionConfig&,const FixedMainSource&,
      const startup::FixedMainView&,const initial_source::PreparedSource&,
      const tl::fea::ShellPhysicalBinding&,GeneralTransactionForecast&,TransactionLimits={}) noexcept;
  static TransactionReport GeneralPreflight(const TransactionConfig&,const MovingMainSource&,
      const initial_source::PreparedSource&,const tl::fea::ShellPhysicalBinding&,
      GeneralTransactionForecast&,TransactionLimits={}) noexcept;
  static TransactionReport GeneralPreflight(const TransactionConfig&,const MixedMovingMainSource&,
      const initial_source::PreparedSource&,const tl::fea::ShellPhysicalBinding&,
      GeneralTransactionForecast&,TransactionLimits={}) noexcept;

  Transaction(const Transaction&)=delete;Transaction& operator=(const Transaction&)=delete;
  Transaction(Transaction&&)=delete;Transaction& operator=(Transaction&&)=delete;
  TransactionReport Initialize(const TransactionConfig&,const FixedMainSource&,
      tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits={}) noexcept;
  TransactionReport Initialize(const TransactionConfig&,const MovingMainSource&,
      tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits={}) noexcept;
  TransactionReport Initialize(const TransactionConfig&,const MixedMovingMainSource&,
      tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits={}) noexcept;
  TransactionReport GeneralInitialize(const TransactionConfig&,const FixedMainSource&,
      const startup::FixedMainView&,const initial_source::PreparedSource&,
      tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits={}) noexcept;
  TransactionReport GeneralInitialize(const TransactionConfig&,const MovingMainSource&,
      const initial_source::PreparedSource&,tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits={}) noexcept;
  TransactionReport GeneralInitialize(const TransactionConfig&,const MixedMovingMainSource&,
      const initial_source::PreparedSource&,tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits={}) noexcept;
  tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;
  // Distinct native-only group registration; old roster_entry remains valid
  // for the legacy single-native self-contact slot.
  tl::fea::NativeContactRosterEntry native_roster_entry() noexcept;
  // Requires complete physical material assembly into this exact live view.
  // Contact forces use accepted X_n/V and history, not proposed endpoint fields.
  TransactionReport AssembleAccepted(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&) noexcept;
  // Authenticates the completed physical candidate and seals the already staged
  // contact buffers. No contact law is reevaluated at X_(n+1).
  TransactionReport SealCandidate(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&,
      tl::fea::ShellPhysicalScratchParticipationReceipt*) noexcept;
  // One serialized, callback-free operation over the complete registered native
  // roster. Only within this call can identical prepared activity be reused.
  // Receipts publish together; failure revokes the common attempt. Standalone
  // SealCandidate remains a fresh query, including repeated same-attempt calls.
  static TransactionGroupReport SealCandidateGroup(Transaction* const* members,
      std::size_t count,tl::fea::ShellBatchPublication&,tl::fea::FENodalState&,
      const tl::fea::NodalTrialToken&,const tl::fea::NodalPreparedView&,
      const tl::fea::ShellPhysicalDiagnostics&,
      tl::fea::ShellPhysicalScratchParticipationReceipt* receipts,
      std::size_t receipt_count) noexcept;
  void DiscardTrial() noexcept;
  tl::fea::NativeContactPublicationSnapshot accepted() const noexcept;
  TransactionReport CopyAccepted(AcceptedContactBuffer,
      tl::fea::NativeContactPublicationSnapshot*) const noexcept;
  TransactionSourceInfo source_info() const noexcept;
  TransactionForecast allocations() const noexcept;
  TransactionDiagnostics last_diagnostics() const noexcept;
  // Historical successful source preparation only; not an accepted physical
  // receipt. Legacy cold Initialize reports unavailable. No GPU row readback.
  TransactionInitializationDiagnostics initialization_diagnostics() const noexcept;
 private:
  struct GroupSealSession;
  TransactionReport SealCandidateImpl(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&,
      tl::fea::ShellPhysicalScratchParticipationReceipt*,GroupSealSession*,std::size_t) noexcept;
  template<class Source> TransactionReport InitializeSource(const TransactionConfig&,const Source&,
      tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits,
      const initial_source::PreparedSource* =nullptr,const startup::FixedMainView* =nullptr) noexcept;
  friend class runtime_qualification::Access;
  struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact::radioss_type25
