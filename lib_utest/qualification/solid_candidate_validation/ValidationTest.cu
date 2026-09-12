// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaRig.h"

namespace solid_validation_test {
template<class Traits> void Fault(std::vector<unsigned char>& bytes,
    const d::FamilyLayout& layout, unsigned kind) {
  auto* value = tl::util::ArenaPointer<d::State<Traits>>(bytes.data(), layout.slab[1]);
  if (kind == 0) value[0].cache.rhs_force_n[0].x = NAN;
  if (kind == 1) value[0].cache.rhs_force_n[Traits::nodes - 1].z = NAN;
  if (kind == 2) value[0].history = {};
  if (kind == 3) value[0].cache.stiffness.translation_n_m = NAN;
  if (kind == 4) *tl::util::ArenaPointer<int>(bytes.data(), layout.status) = 7;
}
inline void CompareFinalize(DeviceRig& rig, s::BatchDiagnostics identity = {}) {
  const auto view = rig.View(0);
  ASSERT_TRUE(rig.Restore(rig.before));
  d::LaunchResultValidation(rig.device, 1, identity.time, identity.epoch, rig.stream);
  d::LaunchFinalizeTest(rig.device, 0, 1, view, identity);
  ASSERT_TRUE(rig.Read(rig.parallel));
  ASSERT_TRUE(rig.Restore(rig.before));
  old::LaunchFinalizeTest(rig.device, 0, 1, view, identity);
  ASSERT_TRUE(rig.Read(rig.serial));
  rig.Compare();
}
TEST_F(SolidCandidateCuda, FrozenFullFoldAllFamilyEarlyLateHistoryAndStatusFaults) {
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
    CompareFinalize(rig);
    ASSERT_FALSE(HasFailure());
    EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status,
        kind == 4 ? s::BatchStatus::ElementFailure : s::BatchStatus::NonfiniteResult);
    for (unsigned f = 0; f < 5; ++f)
      EXPECT_EQ(rig.parallel[layouts[f]->result_valid.offset], f == family ? 0 : 1);
  }
  rig.before = original;
  s::BatchDiagnostics wrong;
  wrong.epoch = 1;
  CompareFinalize(rig, wrong);
  EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::NonfiniteResult);
  rig.before = original;
  CompareFinalize(rig);
  EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::Success);
}
TEST_F(SolidCandidateCuda, OrderedWorkOverflowBeatsValidatedLaterFoamFault) {
  DeviceRig rig;
  rig.host.source.Repeat44();
  ASSERT_TRUE(rig.Initialize());
  rig.before = rig.serial;
  auto* rear = tl::util::ArenaPointer<d::State<d::Traits18Law44>>(
      rig.before.data(), rig.host.layout.solid18_law44.slab[1]);
  rear[0].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  rear[1].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  Fault<d::Traits18Law90>(rig.before, rig.host.layout.solid18_law90, 1);
  CompareFinalize(rig);
  const auto& control = DeviceRig::Header(rig.parallel).control;
  EXPECT_EQ(control.status, s::BatchStatus::NonfiniteResult);
  EXPECT_EQ(control.family, s::Family::Solid18Law44);
  EXPECT_EQ(control.parent, 1u);
  EXPECT_EQ(control.diagnostics.parent_count[4], 0u);
}
template<class Traits> void Unavailable(d::Storage& state, int* status,
    std::uint8_t* flags, std::size_t count) {
  auto& family = d::FamilyStorage<Traits>(state);
  family.count = count;
  family.status = status;
  family.result_valid = flags;
}
TEST_F(SolidCandidateCuda, GridStrideOverwritesEveryFailedRowWithoutHistoryDereference) {
  const std::size_t count = d::candidate_blocks * d::candidate_threads + 17;
  const std::size_t status_bytes = count * sizeof(int);
  const std::size_t total = sizeof(d::Storage) + status_bytes + 5 * count;
  unsigned char* arena = nullptr;
  cudaStream_t stream = nullptr;
  ASSERT_TRUE(Cuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking)));
  ASSERT_TRUE(Cuda(cudaMalloc(reinterpret_cast<void**>(&arena), total)));
  auto* device = reinterpret_cast<d::Storage*>(arena);
  auto* statuses = reinterpret_cast<int*>(arena + sizeof(d::Storage));
  auto* flags = arena + sizeof(d::Storage) + status_bytes;
  d::Storage host;
  Unavailable<d::Traits18>(host, statuses, flags, count);
  Unavailable<d::Traits24>(host, statuses, flags + count, count);
  Unavailable<d::Traits6z>(host, statuses, flags + 2 * count, count);
  Unavailable<d::Traits18Law44>(host, statuses, flags + 3 * count, count);
  Unavailable<d::Traits18Law90>(host, statuses, flags + 4 * count, count);
  ASSERT_TRUE(Cuda(cudaMemcpyAsync(device, &host, sizeof(host), cudaMemcpyHostToDevice, stream)));
  ASSERT_TRUE(Cuda(cudaMemsetAsync(statuses, 1, status_bytes, stream)));
  for (int sentinel : {1, 255}) {
    ASSERT_TRUE(Cuda(cudaMemsetAsync(flags, sentinel, 5 * count, stream)));
    d::LaunchResultValidation(device, 1, 7, 999, stream);
    std::vector<std::uint8_t> result(5 * count);
    ASSERT_TRUE(Cuda(cudaMemcpyAsync(result.data(), flags, result.size(), cudaMemcpyDeviceToHost, stream)));
    ASSERT_TRUE(Cuda(cudaStreamSynchronize(stream)));
    for (auto value : result) EXPECT_EQ(value, 0);
  }
  EXPECT_TRUE(Cuda(cudaFree(arena)));
  EXPECT_TRUE(Cuda(cudaStreamDestroy(stream)));
}
} // namespace solid_validation_test
