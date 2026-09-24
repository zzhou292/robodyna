// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NumericCohortCases.h"
#include "CopyFault.h"
#include <limits>
namespace native_gpu_cohort_test {
TEST(NativeGpuNumericCohortCuda, Windows257And4097KeepOriginal256SliceReportsAndRows) {
  native_gpu_test::Streams streams;
  for (std::size_t count : {257u, 4097u}) {
    SCOPED_TRACE(count);
    const auto source = Roster(count);
    auto limits = Limits(source, 256, 4096);
    auto cpu = test::Owner(limits.native);
    c::RepresentedIntervalCrossingGpu legacy, cached;
    ASSERT_EQ(cached.Initialize(limits, streams.first).native.status, S::Ok);
    limits.numeric_cohort_pairs = 0;
    ASSERT_EQ(legacy.Initialize(limits, streams.first).native.status, S::Ok);
    std::vector<c::RepresentedIntervalResult> expected(count), old_rows(count), rows(count);
    const auto reference = CertifyCohort(cpu, source, 256, expected);
    const auto old = CertifyCohort(legacy, source, 256, old_rows, streams.first);
    const auto actual = CertifyCohort(cached, source, 256, rows, streams.first);
    ASSERT_EQ(actual.native.status, S::Ok);
    Equal(reference, old); Equal(reference, actual);
    test::SameView(cpu.results(), cached.results());
    test::OneFullAuthentication(actual.native, source.paths.size());
    EXPECT_EQ(actual.native.completed_batches, (count + 255) / 256);
    EXPECT_EQ(actual.device.numeric_cohorts, (count + 4095) / 4096);
    EXPECT_EQ(actual.device.batches, actual.device.numeric_cohorts);
    EXPECT_EQ(old.device.batches, (count + 255) / 256);
    EXPECT_EQ(actual.device.scene_uploads, 1u);
    EXPECT_EQ(actual.device.device_pairs, count - count / 4);
    EXPECT_EQ(actual.device.host_pairs, count / 4);
    EXPECT_EQ(actual.device.consumed_device_pairs, actual.device.device_pairs);
  }
}

TEST(NativeGpuNumericCohortCuda, NonmultipleCapacityAlignsWindowsWithoutRepeatedPrefetch) {
  native_gpu_test::Streams streams;
  const auto source = Roster(17);
  const auto limits = Limits(source, 4, 6);
  auto cpu = test::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  std::vector<c::RepresentedIntervalResult> expected(17), rows(17);
  const auto actual = CertifyCohort(gpu, source, 4, rows, streams.first);
  Equal(CertifyCohort(cpu, source, 4, expected), actual);
  ASSERT_EQ(actual.native.status, S::Ok);
  EXPECT_EQ(actual.native.completed_batches, 5u);
  EXPECT_EQ(actual.device.numeric_cohorts, 4u);
  EXPECT_EQ(actual.device.batches, 4u);
  EXPECT_EQ(actual.device.device_pairs, 13u);
  EXPECT_EQ(actual.device.host_pairs, 4u);
  EXPECT_EQ(actual.device.consumed_device_pairs, 13u);
}

TEST(NativeGpuNumericCohortCuda, LateNativeWorkFailureSuppressesPrefetchedLaterSlices) {
  native_gpu_test::Streams streams;
  auto source = Roster(6);
  auto limits = Limits(source, 1, 6);
  limits.native.max_total_work = 1;
  auto cpu = test::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  std::vector<c::RepresentedIntervalResult> expected(6), rows(6);
  const auto actual = CertifyCohort(gpu, source, 1, rows, streams.first);
  Equal(CertifyCohort(cpu, source, 1, expected), actual);
  ASSERT_EQ(actual.native.status, S::ResourceLimit);
  EXPECT_EQ(actual.native.completed_pairs, 2u);
  EXPECT_EQ(actual.native.input_pair, 2u);
  EXPECT_EQ(actual.native.prior_work, 2u);
  EXPECT_EQ(actual.device.device_pairs, 5u);
  EXPECT_EQ(actual.device.consumed_device_pairs, 3u);
  EXPECT_EQ(actual.device.host_pairs, 0u);
  EXPECT_EQ(actual.device.numeric_cohorts, 1u);
  EXPECT_EQ(actual.device.fault_cohort_begin, SIZE_MAX);  // Native cap, not a CUDA fault.
  EXPECT_FALSE(actual.native.results.complete);
  test::SameView(cpu.results(), gpu.results());
  for (auto& path : source.paths)
    for (auto& vertex : path.vertices) vertex.endpoint[1] = vertex.endpoint[0];
  const auto retry = CertifyCohort(gpu, source, 1, rows, streams.first);
  Equal(CertifyCohort(cpu, source, 1, expected), retry);
  ASSERT_EQ(retry.native.status, S::Ok);
  EXPECT_EQ(retry.device.scene_uploads, 1u);
  EXPECT_EQ(retry.device.consumed_device_pairs, 5u);
}

TEST(NativeGpuNumericCohortCuda, TypedUnresolvedAndHostFallbackKeepOriginalWorkAndPriority) {
  native_gpu_test::Streams streams;
  for (bool admitted : {false, true}) {
    const auto source = Roster(9, admitted);
    auto limits = Limits(source, 2, 8);
    limits.native.max_work_per_pair = 1;
    auto cpu = test::Owner(limits.native);
    c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
    std::vector<c::RepresentedIntervalResult> expected(9), rows(9);
    const auto actual = CertifyCohort(gpu, source, 2, rows, streams.first);
    Equal(CertifyCohort(cpu, source, 2, expected), actual);
    ASSERT_EQ(actual.native.status, S::Ok);
    EXPECT_EQ(rows[2].classification, c::RepresentedIntervalClassification::Unresolved);
    EXPECT_EQ(rows[2].reason, c::RepresentedIntervalReason::WorkExhausted);
    EXPECT_EQ(rows[2].work, 1u);
    EXPECT_EQ(rows[3].reason, c::RepresentedIntervalReason::UnsupportedMotion);
    EXPECT_EQ(rows[3].work, 0u);
    EXPECT_EQ(actual.device.numeric_cohorts, 2u);
    EXPECT_EQ(actual.device.device_pairs, admitted ? 7u : 0u);
    EXPECT_EQ(actual.device.host_pairs, admitted ? 2u : 9u);
    EXPECT_EQ(actual.device.scene_uploads, admitted ? 1u : 0u);
  }
}

TEST(NativeGpuNumericCohortCuda, ChangedAndMalformedScenesCannotReuseCachedAuthority) {
  native_gpu_test::Streams streams;
  auto source = Roster(5);
  const auto limits = Limits(source, 2, 4);
  auto cpu = test::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  std::vector<c::RepresentedIntervalResult> expected(5), rows(5);
  Equal(CertifyCohort(cpu, source, 2, expected), CertifyCohort(gpu, source, 2, rows, streams.first));
  for (auto& vertex : source.paths[1].vertices)
    vertex.endpoint[0].z = vertex.endpoint[1].z = 8;
  const auto changed = CertifyCohort(gpu, source, 2, rows, streams.first);
  Equal(CertifyCohort(cpu, source, 2, expected), changed);
  ASSERT_EQ(changed.native.status, S::Ok);
  EXPECT_EQ(rows[0].classification, c::RepresentedIntervalClassification::CertifiedCrossingContact);
  EXPECT_EQ(changed.device.scene_uploads, 1u);
  const auto prior = gpu.results();
  const auto saved = native_gpu_test::Copy(prior);
  source.paths.back().vertices[0].endpoint[1].x = std::numeric_limits<double>::infinity();
  source.pairs.resize(1);
  const auto malformed = CertifyCohort(gpu, source, 2, rows, streams.first);
  Equal(CertifyCohort(cpu, source, 2, expected), malformed);
  EXPECT_NE(malformed.native.status, S::Ok);
  EXPECT_EQ(malformed.device.status, D::NotInvoked);
  native_gpu_test::Preserved(prior, saved, gpu.results());
}

TEST(NativeGpuNumericCohortCuda, AllHostInitialWindowDoesNotPreventLaterEligibleSceneUpload) {
  native_gpu_test::Streams streams;
  auto source = Roster(9);
  for (unsigned i = 1; i <= 4; ++i) source.paths[i].motion = c::RepresentedMotion::RigidArc;
  const auto limits = Limits(source, 2, 4);
  auto cpu = test::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  std::vector<c::RepresentedIntervalResult> expected(9), rows(9);
  const auto actual = CertifyCohort(gpu, source, 2, rows, streams.first);
  Equal(CertifyCohort(cpu, source, 2, expected), actual);
  ASSERT_EQ(actual.native.status, S::Ok);
  EXPECT_EQ(actual.device.numeric_cohorts, 3u);
  EXPECT_EQ(actual.device.scene_uploads, 1u);
  EXPECT_EQ(actual.device.batches, 2u);
  EXPECT_EQ(actual.device.device_pairs, 4u);
  EXPECT_EQ(actual.device.consumed_device_pairs, 4u);
  EXPECT_EQ(actual.device.host_pairs, 5u);
}

TEST(NativeGpuNumericCohortCuda, TooSmallCohortRejectsWithoutChangingNativeSliceOrPublication) {
  native_gpu_test::Streams streams;
  auto source = Roster(3);
  const auto limits = Limits(source, 2, 1);
  c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  auto prefix = source; prefix.pairs.resize(1);
  std::vector<c::RepresentedIntervalResult> rows(3);
  ASSERT_EQ(CertifyCohort(gpu, prefix, 2, rows, streams.first).native.status, S::Ok);
  const auto prior = gpu.results(); const auto values = native_gpu_test::Copy(prior);
  const auto rejected = CertifyCohort(gpu, source, 2, rows, streams.first);
  EXPECT_EQ(rejected.native.status, S::ResourceLimit);
  EXPECT_EQ(rejected.native.native_report.input_pairs, 2u);
  EXPECT_EQ(rejected.native.completed_pairs, 0u);
  EXPECT_EQ(rejected.device.status, D::NotInvoked);
  EXPECT_EQ(rejected.device.batches, 0u);
  native_gpu_test::Preserved(prior, values, gpu.results());
  EXPECT_EQ(CertifyCohort(gpu, prefix, 2, rows, streams.first).native.status, S::Ok);
}

TEST(NativeGpuNumericCohortCuda, EmptyCallRevokesCacheAndOwnedScratchStillRejects) {
  native_gpu_test::Streams streams;
  auto source = Roster(1);
  const auto limits = Limits(source, 1, 4);
  auto cpu = test::Owner(limits.native);
  c::RepresentedIntervalCrossingGpu gpu;
  ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
  std::vector<c::RepresentedIntervalResult> expected(1), rows(1);
  Equal(CertifyCohort(cpu, source, 1, expected), CertifyCohort(gpu, source, 1, rows, streams.first));
  const auto prior = gpu.results(); const auto values = native_gpu_test::Copy(prior);
  const auto alias = batch::DeviceBatchAccess::Certify(gpu, source.paths.data(), source.paths.size(),
      source.pairs.data(), 1, 1, const_cast<c::RepresentedIntervalResult*>(prior.data), prior.count, streams.first);
  EXPECT_EQ(alias.native.status, S::InvalidInput);
  EXPECT_EQ(alias.device.status, D::NotInvoked);
  native_gpu_test::Preserved(prior, values, gpu.results());
  source.pairs.clear(); expected.clear(); rows.clear();
  const auto empty = CertifyCohort(gpu, source, 1, rows, streams.first);
  Equal(CertifyCohort(cpu, source, 1, expected), empty);
  EXPECT_EQ(empty.device.numeric_cohorts, 0u);
  EXPECT_EQ(empty.device.scene_uploads, 0u);
  EXPECT_EQ(empty.device.batches, 0u);
  EXPECT_TRUE(gpu.results().complete); EXPECT_EQ(gpu.results().count, 0u);
}

TEST(NativeGpuNumericCohortCuda, CudaFaultPreservesOnlyTheLastPreCohortPublicationAndPoisonsOwner) {
  native_gpu_test::Streams streams;
  for (std::size_t successful_copies : {0u, 2u}) {
    SCOPED_TRACE(successful_copies);
    const auto source = Roster(5);
    const auto limits = Limits(source, 1, 2);
    auto cpu = test::Owner(limits.native);
    c::RepresentedIntervalCrossingGpu gpu;
    ASSERT_EQ(gpu.Initialize(limits, streams.first).native.status, S::Ok);
    auto prefix = source; prefix.pairs.resize(1);
    std::vector<c::RepresentedIntervalResult> expected(5), rows(5);
    Equal(CertifyCohort(cpu, prefix, 1, expected), CertifyCohort(gpu, prefix, 1, rows, streams.first));
    batch::DeviceBatchReport failed;
    {
      native_gpu_copy_fault::Scope inject(successful_copies);
      failed = CertifyCohort(gpu, source, 1, rows, streams.first);
    }
    EXPECT_EQ(failed.native.status, S::ResourceLimit);
    EXPECT_EQ(failed.device.status, D::DeviceFailure);
    EXPECT_FALSE(failed.native.results.complete);
    const auto completed = successful_copies ? 2u : 0u;
    EXPECT_EQ(failed.native.completed_pairs, completed);
    EXPECT_EQ(failed.device.fault_cohort_begin, completed);
    EXPECT_EQ(failed.device.fault_cohort_count, 2u);
    EXPECT_EQ(failed.device.fault_pair_ordinal, SIZE_MAX);
    if (completed) {
      prefix.pairs = {source.pairs[0], source.pairs[1]};
      ASSERT_EQ(CertifyCohort(cpu, prefix, 1, expected).status, S::Ok);
    }
    test::SameView(cpu.results(), gpu.results());
    const auto prior = gpu.results(); const auto values = native_gpu_test::Copy(prior);
    const auto retry = CertifyCohort(gpu, source, 1, rows, streams.first);
    EXPECT_EQ(retry.device.status, D::DeviceFailure);
    EXPECT_EQ(retry.device.batches, 0u);
    native_gpu_test::Preserved(prior, values, gpu.results());
  }
}
}  // namespace native_gpu_cohort_test
