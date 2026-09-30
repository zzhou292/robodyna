// SPDX-License-Identifier: MIT
#pragma once

#include "lib_src/collision/self_contact_transaction/FinalizedCoverageLedger.h"
#include <stdexcept>

namespace tlfea::contact::self_contact_transaction {

// Owning test seam only. Production construction is private to the transaction.
// Tests permit equal feature keys and arbitrary source order to preserve the raw
// oracle's malformed/duplicate-owner cases; only monotone prefix order is needed
// by the search. This is not a physical receipt or a runtime construction API.
struct FinalizedCoverageLedgerTestAccess {
  static FinalizedCoverageLedger Checked(const AcceptedEventCertificate* data,
                                        std::size_t count) {
    if (count && !data) throw std::invalid_argument("Missing test ledger");
    for (std::size_t i = 1; i < count; ++i)
      if (fixed_triangle_features::Compare(data[i - 1].event.feature,
                                           data[i].event.feature) > 0)
        throw std::invalid_argument("Unsorted test ledger");
    return FinalizedCoverageLedger(data, count);
  }
};

}  // namespace tlfea::contact::self_contact_transaction
