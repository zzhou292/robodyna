// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../represented_interval_crossing/BatchAssertions.h"
#include "../represented_interval_crossing/BatchFixture.h"
#include "lib_src/collision/represented_interval_crossing/DeviceBatch.h"
#include "lib_src/collision/represented_interval_crossing/DeviceExecution.h"
#include "WideStorageCases.h"
#include <limits>
#include <type_traits>

namespace {
namespace c = tlfea::contact;
namespace batch = c::represented_interval_crossing;
namespace test = represented_interval_test;
using S = c::RepresentedIntervalStatus;
using D = c::RepresentedIntervalDeviceStatus;
static_assert(!std::is_default_constructible_v<batch::AuthenticatedScene>);
static_assert(!std::is_copy_constructible_v<batch::AuthenticatedScene>);
static_assert(!std::is_move_constructible_v<batch::AuthenticatedScene>);
static_assert(!std::is_default_constructible_v<batch::AuthenticatedWork>);
static_assert(!std::is_copy_constructible_v<batch::AuthenticatedWork>);

void AdmitCoordinates(test::Roster& roster) {
  // Exact common offset preserves the fixture while removing stored zero
  // exponents from the proved B<=125 domain. No production source special case.
  for (auto& path : roster.paths)
    for (auto& vertex : path.vertices)
      for (auto& point : vertex.endpoint) {
        point.x += 8; point.y += 8; point.z += 8;
      }
}
batch::BatchReport Cpu(c::RepresentedIntervalCrossing& owner, const test::Roster& roster,
    std::size_t capacity, std::vector<c::RepresentedIntervalResult>& scratch) {
  return batch::BatchAccess::Certify(owner, roster.paths.data(), roster.paths.size(),
      roster.pairs.data(), roster.pairs.size(), capacity, scratch.data(), scratch.size());
}
batch::DeviceBatchReport Gpu(c::RepresentedIntervalCrossingGpu& owner, const test::Roster& roster,
    std::size_t capacity, std::vector<c::RepresentedIntervalResult>& scratch, cudaStream_t stream) {
  return batch::DeviceBatchAccess::Certify(owner, roster.paths.data(), roster.paths.size(),
      roster.pairs.data(), roster.pairs.size(), capacity, scratch.data(), scratch.size(), stream);
}
void Equal(const batch::BatchReport& cpu, const batch::DeviceBatchReport& gpu) {
  test::SameBatch(cpu, gpu.native);
  test::SameRosterWork(cpu.path_roster_work, gpu.native.path_roster_work);
}
class NativeGpuCompound : public ::testing::Test {
 protected:
  void SetUp() override { ASSERT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess); }
  void TearDown() override { if (stream) EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess); }
  void Initialize(c::RepresentedIntervalCrossingGpu& owner,
      c::RepresentedIntervalLimits native, unsigned workers = 128) {
    c::RepresentedIntervalGpuLimits limits;
    limits.native = native;
    limits.device_workers = workers;
    ASSERT_EQ(owner.Initialize(limits, stream).native.status, S::Ok);
  }
  cudaStream_t stream = nullptr;
};

TEST_F(NativeGpuCompound, OneAuthenticationAndUploadAcross256And257Boundary) {
  for (unsigned workers : {1u, 37u, 128u})
    for (std::size_t count : {256u, 257u}) {
      SCOPED_TRACE(::testing::Message() << workers << ":" << count);
      test::Roster roster(count);
      AdmitCoordinates(roster);
      const auto limits = test::Limits(roster, 256);
      auto cpu = test::Owner(limits);
      c::RepresentedIntervalCrossingGpu gpu;
      Initialize(gpu, limits, workers);
      std::vector<c::RepresentedIntervalResult> expected(count), actual(count);
      const auto before = Cpu(cpu, roster, 256, expected);
      const auto after = Gpu(gpu, roster, 256, actual, stream);
      ASSERT_EQ(after.native.status, S::Ok);
      Equal(before, after);
      test::SameView(cpu.results(), gpu.results());
      test::OneFullAuthentication(after.native, roster.paths.size());
      EXPECT_EQ(after.device.status, D::Ok);
      EXPECT_EQ(after.device.scene_uploads, 1u);
      EXPECT_EQ(after.device.batches, (count + 255) / 256);
      EXPECT_EQ(after.device.host_pairs, count / 4);
      EXPECT_EQ(after.device.device_pairs, count - count / 4);
    }
}

TEST_F(NativeGpuCompound, WideOnlyAndInitiallyWideSlicesDoNotUploadEarly) {
  for (bool delay : {false, true}) {
    test::Roster roster(5);
    if (delay) {
      AdmitCoordinates(roster);
      roster.paths[1].motion = roster.paths[2].motion = c::RepresentedMotion::RigidArc;
    } else native_gpu_test::RequireNonzeroWideStorage(roster.paths);
    const auto limits = test::Limits(roster, 2);
    auto cpu = test::Owner(limits);
    c::RepresentedIntervalCrossingGpu gpu;
    Initialize(gpu, limits);
    std::vector<c::RepresentedIntervalResult> expected(5), actual(5);
    const auto before = Cpu(cpu, roster, 2, expected);
    const auto after = Gpu(gpu, roster, 2, actual, stream);
    ASSERT_EQ(after.native.status, S::Ok);
    Equal(before, after);
    EXPECT_EQ(after.device.scene_uploads, delay ? 1u : 0u);
    EXPECT_EQ(after.device.batches, delay ? 2u : 0u);
    EXPECT_EQ(after.device.device_pairs, delay ? 2u : 0u);
    EXPECT_EQ(after.device.host_pairs, delay ? 3u : 5u);
  }
}

TEST_F(NativeGpuCompound, EmptyCallsAndReusedInputAddressesCannotBorrowOldScene) {
  test::Roster roster(5);
  AdmitCoordinates(roster);
  const auto limits = test::Limits(roster, 2);
  auto cpu = test::Owner(limits);
  c::RepresentedIntervalCrossingGpu gpu;
  Initialize(gpu, limits);
  std::vector<c::RepresentedIntervalResult> expected(5), actual(5);
  Equal(Cpu(cpu, roster, 2, expected), Gpu(gpu, roster, 2, actual, stream));
  for (auto& vertex : roster.paths[1].vertices)
    vertex.endpoint[0].z = vertex.endpoint[1].z = 8;
  auto changed = Gpu(gpu, roster, 2, actual, stream);
  Equal(Cpu(cpu, roster, 2, expected), changed);
  ASSERT_EQ(changed.native.status, S::Ok);
  EXPECT_EQ(actual.front().classification, c::RepresentedIntervalClassification::CertifiedCrossingContact);
  EXPECT_EQ(changed.device.scene_uploads, 1u);
  roster.pairs.clear();
  expected.clear(); actual.clear();
  const auto empty = Gpu(gpu, roster, 2, actual, stream);
  Equal(Cpu(cpu, roster, 2, expected), empty);
  EXPECT_EQ(empty.device.scene_uploads, 0u);
  EXPECT_EQ(empty.device.batches, 0u);
  test::SameView(cpu.results(), gpu.results());
  EXPECT_TRUE(gpu.results().complete);
  EXPECT_EQ(gpu.results().count, 0u);
}

TEST_F(NativeGpuCompound, MalformedUnusedPathsRejectBeforeAnyDeviceWorkAndRetry) {
  test::Roster roster(5);
  AdmitCoordinates(roster);
  const auto limits = test::Limits(roster, 2);
  auto cpu = test::Owner(limits);
  c::RepresentedIntervalCrossingGpu gpu;
  Initialize(gpu, limits);
  std::vector<c::RepresentedIntervalResult> expected(5), actual(5);
  Equal(Cpu(cpu, roster, 2, expected), Gpu(gpu, roster, 2, actual, stream));
  const auto previous = gpu.results();
  const auto bytes = test::Bytes(previous.data, previous.count);
  const auto saved = roster.paths.back();
  roster.pairs.resize(1);
  for (unsigned mode = 0; mode < 2; ++mode) {
    roster.paths.back() = saved;
    if (mode == 0)
      roster.paths.back().vertices[0].endpoint[1].x = std::numeric_limits<double>::infinity();
    else {
      roster.paths.back() = roster.paths[1];
      roster.paths.back().vertices[0].endpoint[1].x += 1;
    }
    const auto failure = Gpu(gpu, roster, 2, actual, stream);
    Equal(Cpu(cpu, roster, 2, expected), failure);
    EXPECT_NE(failure.native.status, S::Ok);
    EXPECT_EQ(failure.device.status, D::NotInvoked);
    EXPECT_EQ(failure.device.scene_uploads, 0u);
    EXPECT_EQ(gpu.results().data, previous.data);
    EXPECT_EQ(test::Bytes(gpu.results().data, gpu.results().count), bytes);
  }
  roster.paths.back() = saved;
  const auto retry = Gpu(gpu, roster, 2, actual, stream);
  Equal(Cpu(cpu, roster, 2, expected), retry);
  ASSERT_EQ(retry.native.status, S::Ok);
  EXPECT_EQ(retry.device.scene_uploads, 1u);
}

TEST_F(NativeGpuCompound, LateWorkFailurePreservesLastSliceAndFreshRetryUpload) {
  test::Roster roster(3);
  AdmitCoordinates(roster);
  auto limits = test::Limits(roster, 1);
  limits.max_total_work = 1;
  auto cpu = test::Owner(limits);
  c::RepresentedIntervalCrossingGpu gpu;
  Initialize(gpu, limits);
  std::vector<c::RepresentedIntervalResult> expected(3), actual(3);
  const auto failure = Gpu(gpu, roster, 1, actual, stream);
  Equal(Cpu(cpu, roster, 1, expected), failure);
  ASSERT_EQ(failure.native.status, S::ResourceLimit);
  EXPECT_EQ(failure.native.completed_pairs, 2u);
  EXPECT_EQ(failure.native.input_pair, 2u);
  EXPECT_GT(failure.native.native_report.rejected_pair_work, 1u);
  EXPECT_EQ(failure.device.scene_uploads, 1u);
  EXPECT_EQ(failure.device.batches, 3u);
  test::SameView(cpu.results(), gpu.results());
  EXPECT_FALSE(failure.native.results.complete);
  for (auto& vertex : roster.paths.back().vertices)
    vertex.endpoint[1] = vertex.endpoint[0];
  const auto retry = Gpu(gpu, roster, 1, actual, stream);
  Equal(Cpu(cpu, roster, 1, expected), retry);
  ASSERT_EQ(retry.native.status, S::Ok);
  EXPECT_EQ(retry.device.scene_uploads, 1u);
}

TEST_F(NativeGpuCompound, OwnedScratchAndFacadeAliasesRejectBeforeBorrowedReads) {
  test::Roster roster(1);
  AdmitCoordinates(roster);
  const auto limits = test::Limits(roster, 1);
  c::RepresentedIntervalCrossingGpu gpu;
  Initialize(gpu, limits);
  std::vector<c::RepresentedIntervalResult> actual(1);
  ASSERT_EQ(Gpu(gpu, roster, 1, actual, stream).native.status, S::Ok);
  const auto previous = gpu.results();
  const auto bytes = test::Bytes(previous.data, previous.count);
  const auto alias = batch::DeviceBatchAccess::Certify(gpu, roster.paths.data(), roster.paths.size(),
      roster.pairs.data(), 1, 1, const_cast<c::RepresentedIntervalResult*>(previous.data), 1, stream);
  EXPECT_EQ(alias.native.status, S::InvalidInput);
  EXPECT_EQ(alias.device.scene_uploads, 0u);
  const auto facade = batch::DeviceBatchAccess::Certify(gpu,
      reinterpret_cast<const c::RepresentedTrianglePath*>(&gpu), 1, nullptr, 0,
      1, nullptr, 0, stream);
  EXPECT_EQ(facade.native.status, S::InvalidInput);
  EXPECT_EQ(facade.device.scene_uploads, 0u);
  EXPECT_EQ(gpu.results().data, previous.data);
  EXPECT_EQ(test::Bytes(gpu.results().data, gpu.results().count), bytes);
  EXPECT_EQ(Gpu(gpu, roster, 1, actual, stream).native.status, S::Ok);
}

TEST_F(NativeGpuCompound, PairOrderAndCapacityErrorsKeepNativePriorityAndPublication) {
  test::Roster roster(5);
  AdmitCoordinates(roster);
  const auto limits = test::Limits(roster, 2);
  auto cpu = test::Owner(limits);
  c::RepresentedIntervalCrossingGpu gpu;
  Initialize(gpu, limits);
  std::vector<c::RepresentedIntervalResult> expected(5), actual(5);
  Equal(Cpu(cpu, roster, 2, expected), Gpu(gpu, roster, 2, actual, stream));
  const auto previous = gpu.results();
  const auto bytes = test::Bytes(previous.data, previous.count);
  for (unsigned mode = 0; mode < 3; ++mode) {
    auto invalid = roster;
    if (mode == 0) std::swap(invalid.pairs[0], invalid.pairs[1]);
    if (mode == 1) invalid.pairs[1] = invalid.pairs[0];
    const auto capacity = mode == 2 ? 3u : 2u;
    const auto failure = Gpu(gpu, invalid, capacity, actual, stream);
    Equal(Cpu(cpu, invalid, capacity, expected), failure);
    EXPECT_NE(failure.native.status, S::Ok);
    EXPECT_EQ(failure.device.status, D::NotInvoked);
    EXPECT_EQ(failure.device.scene_uploads, 0u);
    EXPECT_EQ(gpu.results().data, previous.data);
    EXPECT_EQ(test::Bytes(gpu.results().data, gpu.results().count), bytes);
  }
}
}  // namespace
