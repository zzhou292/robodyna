// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/self_contact_transaction/CrossingExecutor.h"
#include "lib_src/collision/SelfContactTransactionTypes.h"
#include "FacetFilterCudaProbe.h"
#include "../represented_interval_crossing_gpu/NumericCohortCases.h"
#include <limits>

namespace self_contact_crossing_executor_test {
namespace c = tlfea::contact;
namespace sct = c::self_contact_transaction;
namespace fixture = native_gpu_cohort_test;
namespace native = represented_interval_test;
using S = c::RepresentedIntervalStatus;
using D = c::RepresentedIntervalDeviceStatus;

inline c::SelfContactTransactionConfig Config(bool gpu) {
  c::SelfContactTransactionConfig result;
  result.enable_cuda_native_crossing = gpu;
  result.native_crossing_device_workers = 4096;
  result.native_crossing_numeric_cohort_pairs = 4096;
  return result;
}
inline c::SelfContactTransactionLimits ExecutorLimits(const native::Roster& source,
    std::size_t slice) {
  c::SelfContactTransactionLimits result;
  result.crossing = native::Limits(source, slice);
  return result;
}
inline sct::CrossingExecutionReport Certify(sct::CrossingExecutor& owner,
    const native::Roster& source, std::size_t slice,
    std::vector<c::RepresentedIntervalResult>& rows) {
  return owner.Certify(source.paths.data(), source.paths.size(),
      source.pairs.data(), source.pairs.size(), slice, rows.data(), rows.size());
}
inline void Same(const sct::CrossingBatchReport& expected,
    const sct::CrossingExecutionReport& actual) {
  native::SameBatch(expected, actual.native);
  native::SameRosterWork(expected.path_roster_work, actual.native.path_roster_work);
}

TEST(SelfContactCrossingExecutorCuda, SelectedOwnerKeepsAll4097RowsAnd256PublicationSlices) {
  native_gpu_test::Streams streams;
  const auto source = fixture::Roster(4097);
  const auto limits = ExecutorLimits(source, 256);
  auto oracle = native::Owner(limits.crossing);
  std::vector<c::RepresentedIntervalResult> expected(source.pairs.size());
  const auto reference = fixture::CertifyCohort(oracle, source, 256, expected);
  ASSERT_EQ(reference.status, S::Ok);
  for (bool gpu : {false, true}) {
    SCOPED_TRACE(gpu);
    sct::CrossingExecutor owner;
    const auto config = Config(gpu);
    ASSERT_EQ(owner.Initialize(config, limits, streams.first).native.status, S::Ok);
    EXPECT_EQ(owner.Initialize(config, limits, streams.first).native.status, S::AlreadyInitialized);
    std::vector<c::RepresentedIntervalResult> rows(source.pairs.size());
    for (unsigned retry = 0; retry != 2; ++retry) {
      SCOPED_TRACE(retry);
      const auto actual = Certify(owner, source, 256, rows);
      ASSERT_EQ(actual.native.status, S::Ok);
      Same(reference, actual);
      native::SameView(oracle.results(), owner.results());
      EXPECT_EQ(actual.native.completed_batches, 17u);
      native::OneFullAuthentication(actual.native, source.paths.size());
      EXPECT_EQ(actual.device.status, gpu ? D::Ok : D::NotInvoked);
      EXPECT_EQ(actual.device.device_pairs, gpu ? 3073u : 0u);
      EXPECT_EQ(actual.device.consumed_device_pairs, gpu ? 3073u : 0u);
      EXPECT_EQ(actual.device.host_pairs, gpu ? 1024u : 0u);
      EXPECT_EQ(actual.device.numeric_cohorts, gpu ? 2u : 0u);
      EXPECT_EQ(actual.device.batches, gpu ? 2u : 0u);
      EXPECT_EQ(actual.device.scene_uploads, gpu ? 1u : 0u);
    }
  }
}

TEST(SelfContactCrossingExecutorCuda, LateNativeWorkFailurePreservesLastSliceAndRetry) {
  native_gpu_test::Streams streams;
  for (bool gpu : {false, true}) {
    auto source = fixture::Roster(6);
    auto limits = ExecutorLimits(source, 1);
    limits.crossing.max_total_work = 1;
    auto oracle = native::Owner(limits.crossing);
    sct::CrossingExecutor owner;
    ASSERT_EQ(owner.Initialize(Config(gpu), limits, streams.first).native.status, S::Ok);
    std::vector<c::RepresentedIntervalResult> expected(6), rows(6);
    const auto reference = fixture::CertifyCohort(oracle, source, 1, expected);
    const auto actual = Certify(owner, source, 1, rows);
    ASSERT_EQ(actual.native.status, S::ResourceLimit);
    Same(reference, actual);
    EXPECT_EQ(actual.native.completed_pairs, 2u);
    EXPECT_EQ(actual.native.input_pair, 2u);
    EXPECT_EQ(actual.native.prior_work, 2u);
    EXPECT_FALSE(actual.native.results.complete);
    native::SameView(oracle.results(), owner.results());
    EXPECT_EQ(actual.device.device_pairs, gpu ? 5u : 0u);
    EXPECT_EQ(actual.device.consumed_device_pairs, gpu ? 3u : 0u);
    EXPECT_EQ(actual.device.fault_cohort_begin, SIZE_MAX);
    c::SelfContactTransactionReport physical;
    sct::DescribeCrossingDeviceFailure(actual.device, physical);
    EXPECT_EQ(physical.crossing_device_status, D::NotInvoked);
    for (auto& path : source.paths)
      for (auto& vertex : path.vertices) vertex.endpoint[1] = vertex.endpoint[0];
    const auto retry = Certify(owner, source, 1, rows);
    Same(fixture::CertifyCohort(oracle, source, 1, expected), retry);
    ASSERT_EQ(retry.native.status, S::Ok);
    native::SameView(oracle.results(), owner.results());
  }
}

TEST(SelfContactCrossingExecutorCuda, FailedUnusedSourceValidationCannotReusePreviousGpuScene) {
  native_gpu_test::Streams streams;
  auto source = fixture::Roster(5);
  const auto limits = ExecutorLimits(source, 2);
  auto oracle = native::Owner(limits.crossing);
  sct::CrossingExecutor owner;
  ASSERT_EQ(owner.Initialize(Config(true), limits, streams.first).native.status, S::Ok);
  std::vector<c::RepresentedIntervalResult> expected(5), rows(5);
  Same(fixture::CertifyCohort(oracle, source, 2, expected), Certify(owner, source, 2, rows));
  const auto prior = owner.results();
  const auto saved = native_gpu_test::Copy(prior);
  const auto endpoint = source.paths.back().vertices[0].endpoint[1];
  source.paths.back().vertices[0].endpoint[1].x = std::numeric_limits<double>::infinity();
  source.pairs.resize(1);
  const auto failed = Certify(owner, source, 2, rows);
  Same(fixture::CertifyCohort(oracle, source, 2, expected), failed);
  EXPECT_NE(failed.native.status, S::Ok);
  EXPECT_EQ(failed.device.status, D::NotInvoked);
  EXPECT_EQ(failed.device.batches, 0u);
  native_gpu_test::Preserved(prior, saved, owner.results());
  source.paths.back().vertices[0].endpoint[1] = endpoint;
  const auto retry = Certify(owner, source, 2, rows);
  Same(fixture::CertifyCohort(oracle, source, 2, expected), retry);
  EXPECT_EQ(retry.native.status, S::Ok);
  EXPECT_EQ(retry.device.scene_uploads, 1u);
}

TEST(SelfContactCrossingExecutorCuda, InvalidGpuStartupIsTypedAndCannotSilentlyUseCpu) {
  const auto source = fixture::Roster(1);
  const auto limits = ExecutorLimits(source, 1);
  sct::CrossingExecutor owner;
  const auto failed = owner.Initialize(Config(true), limits, nullptr);
  EXPECT_EQ(failed.native.status, S::InvalidInput);
  EXPECT_EQ(failed.device.status, D::InvalidInput);
  c::SelfContactTransactionReport physical;
  sct::DescribeCrossingDeviceFailure(failed.device, physical);
  EXPECT_EQ(physical.crossing_device_status, D::InvalidInput);
  EXPECT_EQ(physical.crossing_fault_cohort_begin, SIZE_MAX);
  EXPECT_EQ(physical.crossing_fault_pair_ordinal, SIZE_MAX);
  std::vector<c::RepresentedIntervalResult> rows(1);
  EXPECT_EQ(Certify(owner, source, 1, rows).native.status, S::NotInitialized);
  EXPECT_FALSE(owner.results().complete);
}

TEST(SelfContactCrossingExecutorCuda, CudaCopyFailureRetainsPublicationAndPoisonsSelectedOwner) {
  native_gpu_test::Streams streams;
  const auto source = fixture::Roster(5);
  const auto limits = ExecutorLimits(source, 1);
  sct::CrossingExecutor owner;
  ASSERT_EQ(owner.Initialize(Config(true), limits, streams.first).native.status, S::Ok);
  auto prefix = source;
  prefix.pairs.resize(1);
  std::vector<c::RepresentedIntervalResult> rows(5);
  ASSERT_EQ(Certify(owner, prefix, 1, rows).native.status, S::Ok);
  const auto prior = owner.results();
  const auto saved = native_gpu_test::Copy(prior);
  facet_filter_cuda_probe::FailNextHostToDeviceCopy();
  const auto failed = Certify(owner, source, 1, rows);
  EXPECT_EQ(failed.native.status, S::ResourceLimit);
  EXPECT_EQ(failed.device.status, D::DeviceFailure);
  EXPECT_EQ(failed.native.completed_pairs, 0u);
  EXPECT_FALSE(failed.native.results.complete);
  EXPECT_EQ(failed.device.fault_cohort_begin, 0u);
  EXPECT_EQ(failed.device.fault_cohort_count, 5u);
  EXPECT_EQ(failed.device.fault_pair_ordinal, SIZE_MAX);
  native_gpu_test::Preserved(prior, saved, owner.results());
  c::SelfContactTransactionReport physical;
  sct::DescribeCrossingDeviceFailure(failed.device, physical);
  EXPECT_EQ(physical.crossing_device_status, D::DeviceFailure);
  EXPECT_EQ(physical.crossing_fault_cohort_begin, 0u);
  EXPECT_EQ(physical.crossing_fault_cohort_count, 5u);
  EXPECT_EQ(physical.crossing_fault_pair_ordinal, SIZE_MAX);
  EXPECT_EQ(physical.pair, SIZE_MAX);
  const auto retry = Certify(owner, source, 1, rows);
  EXPECT_EQ(retry.device.status, D::DeviceFailure);
  EXPECT_EQ(retry.device.batches, 0u);
  native_gpu_test::Preserved(prior, saved, owner.results());
}
}  // namespace self_contact_crossing_executor_test
