// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {

// Synchronous, borrowed source for the last policy phase. Local topology and
// accepted-ledger success require no exclusion preparation. No callback or
// context survives the call, and at most 64 certificates are materialized.
// A preparation failure makes the geometry result InvalidInput and retains
// the exact typed source failure in report for the transaction caller.
struct PolicyExclusionSource {
  using Prepare = SelfContactTransactionReport (*)(
      void*, AcceptedFeatureExclusionCertificate*, std::size_t,
      std::size_t*) noexcept;
  void* context = nullptr;
  Prepare prepare = nullptr;
  std::size_t capacity = 64;
  SelfContactTransactionReport report;
};

}  // namespace tlfea::contact::self_contact_transaction
