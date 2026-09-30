// SPDX-License-Identifier: AGPL-3.0-or-later
// Retain the preceding five actual CUDA comparisons, including whole startup,
// eight carried intervals, status/overflow ordering and accepted-slab retry.
#include "lib_utest/qualification/solid_candidate_validation/ValidationTest.cu"

namespace tl::fea::solids::batch_detail {
void LaunchOperandFinalizeTest(Storage*, unsigned, unsigned, NodalPreparedView, BatchDiagnostics);
}
namespace solid_validation_test {
inline void CompareOperandFinalize(DeviceRig& rig, s::BatchDiagnostics identity = {},
    double kick_dt = .125) {
  auto view = rig.View(0);
  view.kick_dt = kick_dt;
  ASSERT_TRUE(rig.Restore(rig.before));
  d::LaunchMeasurementValidation(rig.device, 0, 1, view, identity.time, identity.epoch, false, rig.stream);
  d::LaunchOperandFinalizeTest(rig.device, 0, 1, view, identity);
  ASSERT_TRUE(rig.Read(rig.parallel));
  ASSERT_TRUE(rig.Restore(rig.before));
  old::LaunchFinalizeTest(rig.device, 0, 1, view, identity);
  ASSERT_TRUE(rig.Read(rig.serial));
  rig.Compare();
}
TEST_F(SolidCandidateCuda, OperandFoldMatchesFrozenAllFamiliesFaultsSeedsAndFreshRetry) {
  DeviceRig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto original = rig.serial;
  const d::FamilyLayout* layouts[] {&rig.host.layout.solid18, &rig.host.layout.solid24,
      &rig.host.layout.solid6z, &rig.host.layout.solid18_law44, &rig.host.layout.solid18_law90};
  for (unsigned family = 0; family < 5; ++family) for (unsigned kind = 0; kind < 5; ++kind) {
    SCOPED_TRACE(::testing::Message() << family << ':' << kind);
    rig.before = original;
    if (family == 0) Fault<d::Traits18>(rig.before, *layouts[family], kind);
    if (family == 1) Fault<d::Traits24>(rig.before, *layouts[family], kind);
    if (family == 2) Fault<d::Traits6z>(rig.before, *layouts[family], kind);
    if (family == 3) Fault<d::Traits18Law44>(rig.before, *layouts[family], kind);
    if (family == 4) Fault<d::Traits18Law90>(rig.before, *layouts[family], kind);
    s::BatchDiagnostics identity;
    identity.internal_kick_work_j = 7;
    identity.internal_drift_work_j = -7;
    identity.plastic_work_increment_j = 3;
    CompareOperandFinalize(rig, identity, kind % 2 ? -.125 : .125);
    ASSERT_FALSE(HasFailure());
    EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status,
        kind == 4 ? s::BatchStatus::ElementFailure : s::BatchStatus::NonfiniteResult);
  }
  rig.before = original;
  s::BatchDiagnostics stale;
  stale.epoch = 1;
  CompareOperandFinalize(rig, stale);
  EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::NonfiniteResult);
  for (double duration : {0.0, -0.0, .125, -.125}) {
    rig.before = original;
    CompareOperandFinalize(rig, {}, duration);
    ASSERT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::Success);
  }
}
template<class Traits> void SeedAcceptedRhs(std::vector<unsigned char>& bytes,
    const d::FamilyLayout& layout) {
  auto* accepted = tl::util::ArenaPointer<d::State<Traits>>(bytes.data(), layout.slab[0]);
  const double force[]{1e16, 1, -1e16, -0.0, DBL_MIN, -DBL_MIN, .5, -.5};
  for (unsigned n = 0; n < Traits::nodes; ++n)
    accepted[0].cache.rhs_force_n[n] = {force[n], -force[n], force[n]};
}
TEST_F(SolidCandidateCuda, OperandSensitiveAcceptedRhsAndDerivedOverflowMatchFrozenBits) {
  DeviceRig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto original = rig.serial;
  rig.before = original;
  SeedAcceptedRhs<d::Traits18>(rig.before, rig.host.layout.solid18);
  SeedAcceptedRhs<d::Traits24>(rig.before, rig.host.layout.solid24);
  SeedAcceptedRhs<d::Traits6z>(rig.before, rig.host.layout.solid6z);
  SeedAcceptedRhs<d::Traits18Law44>(rig.before, rig.host.layout.solid18_law44);
  SeedAcceptedRhs<d::Traits18Law90>(rig.before, rig.host.layout.solid18_law90);
  s::BatchDiagnostics seeded;
  seeded.internal_kick_work_j = 7;
  seeded.internal_drift_work_j = -7;
  seeded.plastic_work_increment_j = 3;
  CompareOperandFinalize(rig, seeded, -.125);
  ASSERT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::Success);

  rig.before = original;
  auto* parent = tl::util::ArenaPointer<d::Traits18::Parent>(
      rig.before.data(), rig.host.layout.solid18.parents);
  auto* accepted = tl::util::ArenaPointer<d::State<d::Traits18>>(
      rig.before.data(), rig.host.layout.solid18.slab[0]);
  auto* trial = tl::util::ArenaPointer<d::State<d::Traits18>>(
      rig.before.data(), rig.host.layout.solid18.slab[1]);
  for (unsigned n = 0; n < d::Traits18::nodes; ++n) {
    const auto x = rig.host.model.domain()->nodes()[parent[0].domain_nodes[n]].position;
    accepted[0].cache.rhs_force_n[n] = {
        x.x < 0 ? -DBL_MAX : x.x > 0 ? DBL_MAX : 0,
        x.y < 0 ? -DBL_MAX : x.y > 0 ? DBL_MAX : 0,
        x.z < 0 ? -DBL_MAX : x.z > 0 ? DBL_MAX : 0};
  }
  CompareOperandFinalize(rig, {}, DBL_MAX);
  const auto& control = DeviceRig::Header(rig.parallel).control;
  EXPECT_EQ(control.status, s::BatchStatus::NonfiniteResult);
  EXPECT_EQ(control.family, s::Family::Solid18);
  EXPECT_EQ(control.parent, 0u);
  EXPECT_EQ(control.diagnostics.native_internal_work_increment_j[0],
      d::Work(trial[0].cache));
}
TEST_F(SolidCandidateCuda, OperandWorkOverflowPrecedesLaterFoamFailureThenRetry) {
  DeviceRig rig;
  rig.host.source.Repeat44();
  ASSERT_TRUE(rig.Initialize());
  const auto original = rig.serial;
  rig.before = original;
  auto* rear = tl::util::ArenaPointer<d::State<d::Traits18Law44>>(
      rig.before.data(), rig.host.layout.solid18_law44.slab[1]);
  rear[0].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  rear[1].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  Fault<d::Traits18Law90>(rig.before, rig.host.layout.solid18_law90, 1);
  CompareOperandFinalize(rig);
  const auto& control = DeviceRig::Header(rig.parallel).control;
  EXPECT_EQ(control.status, s::BatchStatus::NonfiniteResult);
  EXPECT_EQ(control.family, s::Family::Solid18Law44);
  EXPECT_EQ(control.parent, 1u);
  EXPECT_EQ(control.diagnostics.parent_count[4], 0u);
  rig.before = original;
  CompareOperandFinalize(rig);
  EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::Success);
}
template<class Traits> void BindUnavailableOperands(d::Storage& state, unsigned char* base,
    const tl::util::ArenaRegion& status, const tl::util::ArenaRegion& flags,
    const tl::util::ArenaRegion& operands, std::size_t count) {
  auto& family = d::FamilyStorage<Traits>(state);
  family.count = count;
  family.status = tl::util::ArenaPointer<int>(base, status);
  family.result_valid = tl::util::ArenaPointer<std::uint8_t>(base, flags);
  family.measurement = tl::util::ArenaPointer<d::MeasurementOperands<Traits::nodes>>(base, operands);
}
TEST_F(SolidCandidateCuda, OperandGridStrideFailedRowsOverwriteWithoutUnavailableInputReads) {
  const std::size_t count = d::candidate_blocks * d::candidate_threads + 17;
  tl::util::BoundedArenaLayout layout(16 * 1024 * 1024);
  tl::util::ArenaRegion header, status, flags[5], operands[5];
  ASSERT_TRUE(layout.Append<d::Storage>(1, header));
  ASSERT_TRUE(layout.Append<int>(count, status));
  for (unsigned f = 0; f < 5; ++f) {
    ASSERT_TRUE(layout.Append<std::uint8_t>(count, flags[f]));
    if (f == 2) {
      ASSERT_TRUE(layout.Append<d::MeasurementOperands<6>>(count, operands[f]));
    } else {
      ASSERT_TRUE(layout.Append<d::MeasurementOperands<8>>(count, operands[f]));
    }
  }
  unsigned char* arena = nullptr;
  cudaStream_t stream = nullptr;
  ASSERT_TRUE(Cuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking)));
  ASSERT_TRUE(Cuda(cudaMalloc(reinterpret_cast<void**>(&arena), layout.bytes())));
  d::Storage host;
  BindUnavailableOperands<d::Traits18>(host, arena, status, flags[0], operands[0], count);
  BindUnavailableOperands<d::Traits24>(host, arena, status, flags[1], operands[1], count);
  BindUnavailableOperands<d::Traits6z>(host, arena, status, flags[2], operands[2], count);
  BindUnavailableOperands<d::Traits18Law44>(host, arena, status, flags[3], operands[3], count);
  BindUnavailableOperands<d::Traits18Law90>(host, arena, status, flags[4], operands[4], count);
  auto* device = tl::util::ArenaPointer<d::Storage>(arena, header);
  for (int sentinel : {1, 255}) {
    ASSERT_TRUE(Cuda(cudaMemsetAsync(arena, sentinel, layout.bytes(), stream)));
    ASSERT_TRUE(Cuda(cudaMemcpyAsync(device, &host, sizeof(host), cudaMemcpyHostToDevice, stream)));
    // Every status is nonzero. All parent, history, material and view fields are absent.
    d::LaunchMeasurementValidation(device, 0, 1, {}, 7, 999, false, stream);
    std::vector<unsigned char> result(layout.bytes());
    ASSERT_TRUE(Cuda(cudaMemcpyAsync(result.data(), arena, result.size(), cudaMemcpyDeviceToHost, stream)));
    ASSERT_TRUE(Cuda(cudaStreamSynchronize(stream)));
    for (unsigned f = 0; f < 5; ++f) {
      for (std::size_t b = 0; b < flags[f].bytes; ++b) EXPECT_EQ(result[flags[f].offset + b], 0);
      for (std::size_t b = 0; b < operands[f].bytes; ++b) EXPECT_EQ(result[operands[f].offset + b], 0);
    }
  }
  EXPECT_TRUE(Cuda(cudaFree(arena)));
  EXPECT_TRUE(Cuda(cudaStreamDestroy(stream)));
}
} // namespace solid_validation_test
