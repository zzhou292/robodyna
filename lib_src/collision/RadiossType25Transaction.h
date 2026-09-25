// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/runtime/Types.h"
#include "../elements/ShellBatchPublication.h"
#include "../elements/publication/NativeContactPublicationState.h"
#include <memory>
namespace tlfea::contact::radioss_type25 {
namespace runtime_qualification {class Access;}
struct AcceptedContactBuffer {
  NativeGeometryHistory* rows=nullptr;
  int* initial_contact_flags=nullptr;
  std::size_t row_capacity=0;
};
// Concrete fixed-main GPU contact participant. FENodalState remains the sole
// clock and state owner. This stable-address object is noncopyable/nonmovable.
// Serialize every call/read/destruction with the owner and common publisher;
// owner and its borrowed stream must outlive destruction. The publisher may
// destruct first: its typed issuer detaches and all later calls reject before
// dereferencing the former publisher. All staging calls drain.
// Only ShellBatchPublication::CommitPhysical can publish accepted selectors.
class Transaction {
 public:
  Transaction();~Transaction();
  Transaction(const Transaction&)=delete;Transaction& operator=(const Transaction&)=delete;
  Transaction(Transaction&&)=delete;Transaction& operator=(Transaction&&)=delete;
  TransactionReport Initialize(const TransactionConfig&,const FixedMainSource&,
      tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,
      const tl::fea::ShellPhysicalBinding&,const tl::fea::ShellPhysicalParticipants&,
      const tl::fea::ShellPhysicalPublicationIdentity&,TransactionLimits={}) noexcept;
  tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;
  // Requires complete physical material assembly into this exact live view.
  // Contact forces use accepted X_n/V and history, not proposed endpoint fields.
  TransactionReport AssembleAccepted(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalAssemblyView&) noexcept;
  // Authenticates the completed physical candidate and seals the already staged
  // contact buffers. No contact law is reevaluated at X_(n+1).
  TransactionReport SealCandidate(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&,
      tl::fea::ShellPhysicalScratchParticipationReceipt*) noexcept;
  void DiscardTrial() noexcept;
  tl::fea::NativeContactPublicationSnapshot accepted() const noexcept;
  TransactionReport CopyAccepted(AcceptedContactBuffer,
      tl::fea::NativeContactPublicationSnapshot*) const noexcept;
  TransactionSourceInfo source_info() const noexcept;
  TransactionForecast allocations() const noexcept;
  TransactionDiagnostics last_diagnostics() const noexcept;
 private:
  friend class runtime_qualification::Access;
  struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact::radioss_type25
