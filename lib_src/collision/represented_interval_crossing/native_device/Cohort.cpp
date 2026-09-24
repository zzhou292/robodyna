// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Workspace.h"
#include "../NativeStorageDomain.h"
#include "../native/Identity.h"

namespace tlfea::contact::represented_interval_crossing::native_device {
RepresentedIntervalReport Workspace::ExecuteCohort(const AuthenticatedWork& work) noexcept {
  auto& scene = work.scene_;
  auto& cohort = *scene.cohort_;
  if (!cache_ || !scene.ordered_pairs_ || !cohort.count_ ||
      cohort.count_ > layout_.host_cache.count || cohort.count_ > layout_.jobs.count ||
      cohort.begin_ > scene.ordered_pair_count_ ||
      cohort.count_ > scene.ordered_pair_count_ - cohort.begin_ ||
      work.first_ordinal_ < cohort.begin_ ||
      work.first_ordinal_ - cohort.begin_ > cohort.count_ ||
      work.pair_count() > cohort.count_ - (work.first_ordinal_ - cohort.begin_) ||
      !Disjoint(scene.ordered_pairs_, scene.ordered_pair_count_ * sizeof(*scene.ordered_pairs_)))
    return InvalidPublication(work, "Native CUDA numerical cohort lease or retained capacity is inconsistent", SIZE_MAX);

  if (!cohort.computed_) {
    std::size_t jobs = 0;
    ++report_.numeric_cohorts;
    for (std::size_t i = 0; i < cohort.count_; ++i) {
      const auto ordinal = cohort.begin_ + i;
      const auto pair = CanonicalizePair(work.paths(), scene.ordered_pairs_[ordinal], ordinal,
          [](const auto& a, const auto& b) { return native::Compare(a, b); });
      auto& cached = cache_[i];
      cached.device_complete = false;
      cached.result = {};
      cached.result.key = pair.key;
      if (NativeStorageDomain::FromPaths(work.paths()[pair.first], work.paths()[pair.second],
                                       work.limits().max_depth).eligible())
        jobs_[jobs++] = {pair, i};
    }
    report_.device_pairs += jobs;
    const auto submitted = Submit(work, jobs);
    if (submitted.status != RepresentedIntervalStatus::Ok) return submitted;
    for (std::size_t i = 0; i < jobs; ++i) {
      auto& cached = cache_[jobs_[i].ordinal];
      native::StoreResult(results_[i].value, &cached.result);
      cached.device_complete = true;
    }
    // The native lexical window is the only cache authority. It cannot survive
    // the outer call, and no caller pointer or permission is stored in Workspace.
    cohort.computed_ = true;
  }

  const auto first = work.first_ordinal_ - cohort.begin_;
  for (std::size_t i = 0; i < work.pair_count(); ++i)
    if (native::Compare(cache_[first + i].result.key, work.pairs()[i].key) != 0)
      return InvalidPublication(work, "Native CUDA cache identity differs from its admitted publication slice",
                                work.first_ordinal_ + i);
  for (std::size_t i = 0; i < work.pair_count(); ++i) {
    const auto& cached = cache_[first + i];
    if (!cached.device_complete) {
      ++report_.host_pairs;
      continue;
    }
    native::StoreResult(cached.result, work.staging() + i);
    work.status()[i].complete = true;
    ++report_.consumed_device_pairs;
  }
  return {};
}
}  // namespace tlfea::contact::represented_interval_crossing::native_device
