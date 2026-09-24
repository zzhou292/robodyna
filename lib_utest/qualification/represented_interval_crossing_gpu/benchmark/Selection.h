// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include "Results.h"
#include "lib_src/collision/represented_interval_crossing/NativeStorageDomain.h"
#include <stdexcept>

namespace native_gpu_benchmark {
struct Selection {
  native_batch_benchmark::Cases cases;
  std::size_t selected_work = 0, omitted_work = 0;
};
// Measurement-only pair selection. Keep every source path and every coordinate
// bit unchanged; this is never passed to a production vehicle contact policy.
inline Selection Select(const native_batch_benchmark::Cases& source,
    const native_batch_benchmark::VerifiedResults& original, unsigned depth,
    bool eligible_only) {
  namespace c = tlfea::contact;
  if (source.pairs.size() != original.rows.size() || source.kind.size() != source.pairs.size())
    throw std::logic_error("Selection requires the complete verified source cohort");
  Selection result;
  result.cases.paths = source.paths;
  for (std::size_t i = 0; i < source.pairs.size(); ++i) {
    const auto pair = source.pairs[i];
    if (!eligible_only || c::represented_interval_crossing::NativeStorageDomain::FromPaths(
        source.paths[pair.first], source.paths[pair.second], depth).eligible()) {
      result.cases.pairs.push_back(pair);
      result.cases.kind.push_back(source.kind[i]);
      result.selected_work += original.rows[i].work;
    } else {
      result.omitted_work += original.rows[i].work;
    }
  }
  if (result.cases.pairs.empty() || result.selected_work + result.omitted_work != original.work)
    throw std::logic_error("Empty or incomplete benchmark selection");
  return result;
}
}  // namespace native_gpu_benchmark
