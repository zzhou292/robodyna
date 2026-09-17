// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactTransactionTypes.h"

namespace tlfea::contact::self_contact_transaction {

class QualificationAccess;

struct QualificationPreparedActivitySummary {
  std::size_t selected = 0;
  std::size_t accepted_active = 0;
  std::size_t prepared_active = 0;
  std::size_t removing = 0;
  std::size_t inactive = 0;
  bool complete = false;
};

// Diagnostic authority only: the prepared activity generation preserves both
// authenticated endpoint activity arrays after consuming the accepted receipt.
// This cannot seal or publish a physical candidate. Validity authenticates
// activity only, even if a later census traversal fails or exhausts capacity;
// callers must separately check report and complete roster publications.
// Discard, publication, or another activity generation invalidates it and
// every view borrowed from it.
class QualificationPreparedCensusReceipt {
 public:
  QualificationPreparedCensusReceipt() noexcept = default;
  bool valid() const noexcept {
    return transaction_ != nullptr && activity_.valid();
  }
  QualificationPreparedActivitySummary activity_summary() const noexcept {
    return valid() ? activity_summary_ : QualificationPreparedActivitySummary{};
  }

 private:
  friend class QualificationAccess;
  const SelfContactTransaction* transaction_ = nullptr;
  SelfContactAcceptedAssemblyReceipt assembly_;
  SelfContactPreparedActivityReceipt activity_;
  QualificationPreparedActivitySummary activity_summary_;
};

}  // namespace tlfea::contact::self_contact_transaction
