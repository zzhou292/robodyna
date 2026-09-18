// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CrossingBatch.h"
#include "../SelfContactTransactionTypes.h"

namespace tlfea::contact::self_contact_transaction {
CrossingBatchReport CertifyCrossingBatches(
    RepresentedIntervalCrossing& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity,
    RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity) noexcept {
  return represented_interval_crossing::BatchAccess::Certify(
      crossing, paths, path_count, pairs, pair_count,
      batch_pair_capacity, scratch, scratch_capacity);
}

SelfContactCrossingDiagnostics CrossingBatchDiagnostics(
    const CrossingBatchReport& batch) noexcept {
  SelfContactCrossingDiagnostics result;
  if (!batch.native_called) return result;
  const auto& native = batch.native_report;
  result.available = true;
  result.batch_pair_offset = batch.batch_offset;
  result.prior_batch_work = batch.prior_work;
  result.input_path = native.input_path;
  result.input_pair = native.input_pair;
  result.input_paths = native.input_paths;
  result.input_pairs = native.input_pairs;
  result.unique_pairs = native.unique_pairs;
  result.certified_separated = native.certified_separated;
  result.certified_crossing_contact = native.certified_crossing_contact;
  result.unresolved = native.unresolved;
  result.admitted_work = native.work;
  result.total_work_limit = native.total_work_limit;
  result.rejected_pair_work = native.rejected_pair_work;
  return result;
}

}  // namespace tlfea::contact::self_contact_transaction
