// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {

// Diagnostic evidence, never a commit or complete-census receipt. All arrays
// borrow the live production attempt and expire when the callback returns.
// facets indexes the FULL motion inventory in canonical immutable-key order;
// report.pair retains its original filtered-chunk diagnostic ordinal.
struct CandidateFailureCapture {
  SelfContactTransaction* transaction = nullptr;
  // Consumed origin receipt; accepted-policy replay must use activity below.
  const SelfContactAcceptedAssemblyReceipt* accepted_assembly = nullptr;
  SelfContactTransactionReport report;
  FixedTrianglePair facets;
  tl::fea::NodalStamp accepted;
  tl::fea::NodalPreparedView prepared;
  PreparedMotionCertificateView motion;
  AcceptedEventCertificateView accepted_events;
  QualificationPreparedCensusReceipt activity;
  // Present only when the terminal production path has this actual value.
  // Work includes the initial nonlinear separation and subsequent coverage;
  // it is not a replay with freshly reset standalone limits.
  bool has_nonlinear_baseline = false;
  NonlinearSeparationResult nonlinear_baseline;
};

// Borrowed output storage is checked before any production operation. The
// callback must copy bounded evidence synchronously and catch its own errors.
// It may replay accepted feature policies using activity, but must not advance,
// discard or reenter the owner/transaction. Nested output-buffer extents and
// their nonaliasing are caller contracts. A callback cannot suppress rollback
// or replace the production report. No callback runs for success, invalid
// observer input or a failure without an authenticated facet pair.
struct CandidateFailureObserver {
  void* context = nullptr;
  std::size_t context_bytes = 0;
  void (*capture)(void*, const CandidateFailureCapture&) noexcept = nullptr;
};

}  // namespace tlfea::contact::self_contact_transaction
