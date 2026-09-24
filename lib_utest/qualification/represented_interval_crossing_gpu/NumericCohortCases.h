// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "GpuFixture.h"
#include "../represented_interval_crossing/BatchAssertions.h"
#include "../represented_interval_crossing/BatchFixture.h"
#include "lib_src/collision/represented_interval_crossing/DeviceBatch.h"
namespace native_gpu_cohort_test {
namespace c = tlfea::contact;
namespace batch = c::represented_interval_crossing;
namespace test = represented_interval_test;
using S = c::RepresentedIntervalStatus;
using D = c::RepresentedIntervalDeviceStatus;
inline test::Roster Roster(std::size_t count, bool admitted = true) {
  test::Roster result(count);
  if (admitted)
    for (auto& path : result.paths)
      for (auto& vertex : path.vertices)
        for (auto& point : vertex.endpoint) {
          point.x += 8; point.y += 8; point.z += 8;
        }
  else native_gpu_test::RequireNonzeroWideStorage(result.paths);
  return result;
}
inline c::RepresentedIntervalGpuLimits Limits(const test::Roster& source,
    std::size_t slice, std::size_t cohort) {
  c::RepresentedIntervalGpuLimits result;
  result.native = test::Limits(source, slice);
  result.numeric_cohort_pairs = cohort;
  return result;
}
inline batch::BatchReport CertifyCohort(c::RepresentedIntervalCrossing& owner,
    const test::Roster& source, std::size_t slice,
    std::vector<c::RepresentedIntervalResult>& output) {
  return batch::BatchAccess::Certify(owner, source.paths.data(), source.paths.size(),
      source.pairs.data(), source.pairs.size(), slice, output.data(), output.size());
}
inline batch::DeviceBatchReport CertifyCohort(c::RepresentedIntervalCrossingGpu& owner,
    const test::Roster& source, std::size_t slice,
    std::vector<c::RepresentedIntervalResult>& output, cudaStream_t stream) {
  return batch::DeviceBatchAccess::Certify(owner, source.paths.data(), source.paths.size(),
      source.pairs.data(), source.pairs.size(), slice, output.data(), output.size(), stream);
}
inline void Equal(const batch::BatchReport& original, const batch::DeviceBatchReport& current) {
  test::SameBatch(original, current.native);
  test::SameRosterWork(original.path_roster_work, current.native.path_roster_work);
}
}  // namespace native_gpu_cohort_test
