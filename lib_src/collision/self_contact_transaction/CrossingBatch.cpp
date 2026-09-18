// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CrossingBatch.h"
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>

namespace tlfea::contact::self_contact_transaction {
namespace {

template <class T>
bool Bytes(const T* data, std::size_t count, std::size_t* bytes) noexcept {
  if (count > SIZE_MAX / sizeof(T) || (count && !data)) return false;
  *bytes = count * sizeof(T);
  return *bytes <= UINTPTR_MAX - reinterpret_cast<std::uintptr_t>(data);
}

RepresentedIntervalPairKey Key(const RepresentedTrianglePath* paths,
                               RepresentedTrianglePair pair) noexcept {
  RepresentedIntervalPairKey key{{paths[pair.first].key,
                                  paths[pair.second].key}};
  if (Compare(key.paths[1], key.paths[0]) < 0)
    std::swap(key.paths[0], key.paths[1]);
  return key;
}

CrossingBatchReport Failure(CrossingBatchReport report,
                           RepresentedIntervalStatus status,
                           const char* message,
                           std::size_t input_pair = SIZE_MAX) noexcept {
  report.status = status;
  report.message = message;
  report.input_pair = input_pair;
  report.results = {};
  return report;
}

}  // namespace

CrossingBatchReport CertifyCrossingBatches(
    RepresentedIntervalCrossing& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity,
    RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity) noexcept {
  using S = RepresentedIntervalStatus;
  CrossingBatchReport report;
  if (!crossing.initialized())
    return Failure(report, S::NotInitialized,
                   "Crossing batch owner is not initialized");
  const auto capacity = crossing.forecast();
  if (!batch_pair_capacity || batch_pair_capacity > capacity.pair_capacity ||
      batch_pair_capacity > capacity.result_capacity ||
      path_count > capacity.path_index_capacity ||
      pair_count > scratch_capacity)
    return Failure(report, S::ResourceLimit,
                   "Crossing batch storage or admission capacity is insufficient");
  std::size_t path_bytes = 0, pair_bytes = 0, scratch_bytes = 0;
  const auto previous = crossing.results();
  std::size_t previous_bytes = 0;
  const auto disjoint = [&](const void* data, std::size_t bytes) {
    return !scratch_bytes || !bytes || tl::fea::trial_identity::Disjoint(
        scratch, scratch_bytes, data, bytes);
  };
  if (!Bytes(paths, path_count, &path_bytes) ||
      !Bytes(pairs, pair_count, &pair_bytes) ||
      !Bytes(scratch, scratch_capacity, &scratch_bytes) ||
      !Bytes(previous.data, previous.count, &previous_bytes) ||
      !disjoint(paths, path_bytes) || !disjoint(pairs, pair_bytes) ||
      !disjoint(previous.data, previous_bytes))
    return Failure(report, S::InvalidInput,
                   "Crossing batch storage aliases inputs or has an invalid range");
  RepresentedIntervalPairKey preceding;
  for (std::size_t pair = 0; pair < pair_count; ++pair) {
    if (pairs[pair].first >= path_count || pairs[pair].second >= path_count)
      return Failure(report, S::InvalidInput,
                     "Crossing batch pair has an invalid path index", pair);
    const auto key = Key(paths, pairs[pair]);
    if (Compare(key.paths[0], key.paths[1]) >= 0 ||
        (pair && Compare(preceding, key) >= 0))
      return Failure(report, S::IdentityMismatch,
                     "Crossing batch pair roster is not strictly canonical", pair);
    preceding = key;
  }

  std::size_t total_work = 0;
  // The empty call still authenticates the complete supplied path roster.
  do {
    const auto count = std::min(batch_pair_capacity,
                               pair_count - report.completed_pairs);
    report.batch_offset = report.completed_pairs;
    report.prior_work = total_work;
    report.native_report = crossing.Certify(
        paths, path_count, pairs ? pairs + report.batch_offset : nullptr, count);
    report.native_called = true;
    const auto& native = report.native_report;
    if (native.status != S::Ok)
      return Failure(report, native.status, native.message,
          native.input_pair < count ? report.batch_offset + native.input_pair
                                    : SIZE_MAX);
    const auto current = crossing.results();
    if (!current.complete || current.count != count ||
        (count && !current.data))
      return Failure(report, S::IdentityMismatch,
                     "Crossing batch native publication is incomplete");
    for (std::size_t pair = 0; pair < count; ++pair) {
      if (Compare(current.data[pair].key,
                  Key(paths, pairs[report.batch_offset + pair])) != 0)
        return Failure(report, S::IdentityMismatch,
                       "Crossing batch native result changed canonical identity",
                       report.batch_offset + pair);
      scratch[report.batch_offset + pair] = current.data[pair];
    }
    if (native.work > SIZE_MAX - total_work)
      return Failure(report, S::ResourceLimit,
                     "Crossing batch complete work count overflowed");
    total_work += native.work;
    report.completed_pairs += count;
    ++report.completed_batches;
  } while (report.completed_pairs < pair_count);
  report.results = {scratch, pair_count, true};
  return report;
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
