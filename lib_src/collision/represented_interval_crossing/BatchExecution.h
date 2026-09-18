// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "Batch.h"

#include <algorithm>

namespace tlfea::contact::represented_interval_crossing::detail {

inline BatchReport Failure(BatchReport report, RepresentedIntervalStatus status,
                           const char* message,
                           std::size_t input_pair = SIZE_MAX) noexcept {
  report.status = status;
  report.message = message;
  report.input_pair = input_pair;
  report.results = {};
  return report;
}

template <class T>
bool Bytes(const T* data, std::size_t count, std::size_t* bytes) noexcept {
  if (count > SIZE_MAX / sizeof(T) || (count && !data)) return false;
  *bytes = count * sizeof(T);
  return *bytes <= UINTPTR_MAX - reinterpret_cast<std::uintptr_t>(data);
}

template <class Compare>
RepresentedIntervalPairKey Key(const RepresentedTrianglePath* paths,
                               RepresentedTrianglePair pair,
                               const Compare& compare) noexcept {
  RepresentedIntervalPairKey key{{paths[pair.first].key,
                                  paths[pair.second].key}};
  if (compare(key.paths[1], key.paths[0]) < 0)
    std::swap(key.paths[0], key.paths[1]);
  return key;
}

// These helpers are called only by the native-owned compound entry. Its
// implementation supplies the fixed callbacks; callers of BatchAccess cannot
// execute arbitrary code between path authentication and later pair slices.
template <class Compare, class Disjoint>
BatchReport ValidateInput(
    bool initialized, RepresentedIntervalForecast capacity,
    RepresentedIntervalResultView previous,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity, RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity, const Compare& compare,
    const Disjoint& range_disjoint) noexcept {
  using S = RepresentedIntervalStatus;
  BatchReport report;
  if (!initialized)
    return Failure(report, S::NotInitialized,
                   "Crossing batch owner is not initialized");
  if (!batch_pair_capacity || batch_pair_capacity > capacity.pair_capacity ||
      batch_pair_capacity > capacity.result_capacity ||
      path_count > capacity.path_index_capacity ||
      pair_count > scratch_capacity)
    return Failure(report, S::ResourceLimit,
                   "Crossing batch storage or admission capacity is insufficient");
  std::size_t path_bytes = 0, pair_bytes = 0, scratch_bytes = 0;
  std::size_t previous_bytes = 0;
  const auto disjoint = [&](const void* data, std::size_t bytes) {
    return !scratch_bytes || !bytes || range_disjoint(
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
    const auto key = Key(paths, pairs[pair], compare);
    if (compare(key.paths[0], key.paths[1]) >= 0 ||
        (pair && compare(preceding, key) >= 0))
      return Failure(report, S::IdentityMismatch,
                     "Crossing batch pair roster is not strictly canonical", pair);
    preceding = key;
  }
  return report;
}

template <class Certify, class View, class Compare>
BatchReport Execute(
    const RepresentedTrianglePath* paths,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity, RepresentedIntervalResult* scratch,
    const Certify& certify, const View& view, const Compare& compare) noexcept {
  using S = RepresentedIntervalStatus;
  BatchReport report;
  std::size_t total_work = 0;
  // Empty input still authenticates all supplied paths and publishes the
  // same complete empty native slice as the original raw call.
  do {
    const auto count = std::min(batch_pair_capacity,
                               pair_count - report.completed_pairs);
    report.batch_offset = report.completed_pairs;
    report.prior_work = total_work;
    report.native_report = certify(
        pairs ? pairs + report.batch_offset : nullptr, count);
    report.native_called = true;
    const auto& native = report.native_report;
    if (native.status != S::Ok)
      return Failure(report, native.status, native.message,
          native.input_pair < count ? report.batch_offset + native.input_pair
                                    : SIZE_MAX);
    const auto current = view();
    if (!current.complete || current.count != count ||
        (count && !current.data))
      return Failure(report, S::IdentityMismatch,
                     "Crossing batch native publication is incomplete");
    for (std::size_t pair = 0; pair < count; ++pair) {
      if (compare(current.data[pair].key,
                  Key(paths, pairs[report.batch_offset + pair], compare)) != 0)
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

} // namespace tlfea::contact::represented_interval_crossing::detail
