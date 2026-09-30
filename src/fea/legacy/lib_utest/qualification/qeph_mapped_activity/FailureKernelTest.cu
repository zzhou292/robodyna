// SPDX-License-Identifier: MIT
#include "FailureValuesFixture.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/ShellMixedSectionStorage.h"
#include "lib_src/elements/failure/ShellFailureArenaLayout.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <new>

namespace qeph_activity_test {
namespace {
constexpr unsigned KernelParents = 257;
struct FailureInput {
  fe::qeph::batch_detail::Storage storage;
  fe::shell_batch_plasticity_detail::MixedDeviceStorage mixed;
  fe::shell_batch_plasticity_detail::FailureDeviceStorage failure;
  fe::ShellSectionLaw law[KernelParents];
  fe::ShellFailurePolicy policy[KernelParents];
  fe::ShellBatchSectionState section[KernelParents];
  fe::ShellBatchFailureState value[KernelParents];
  std::uint32_t first_invalid;
  std::uint8_t active[KernelParents];
};
}
TEST(QephFailureActivityCuda,ParallelFailureValidationMatchesFrozenPoliciesAndFirstParent) {
  FailureInput* device = nullptr;
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&device),sizeof(*device)),cudaSuccess);
  new(device) FailureInput{};
  cudaStream_t stream;
  ASSERT_EQ(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking),cudaSuccess);
  std::array<FailureCase,FailureCases> cases;
  FillFailureCases(cases);
  for (unsigned mode = 0; mode < 5; ++mode) {
    device->storage.model.config.element_count = KernelParents;
    device->storage.assembly.activity = {&device->first_invalid,device->active};
    device->mixed.law = device->law;
    device->mixed.plastic.section[0] = device->section;
    device->failure.policy = device->policy;
    device->failure.state[0] = device->value;
    device->first_invalid = UINT32_MAX;
    const double time = mode == 4 ? std::numeric_limits<double>::quiet_NaN() : 1.;
    std::array<bool,KernelParents> valid;
    std::uint32_t first = UINT32_MAX;
    for (unsigned p = 0; p < KernelParents; ++p) {
      unsigned index = mode == 3 ? p % FailureCases : p % 9;
      if ((mode == 1 || mode == 2) && p == KernelParents-1) index = 19;
      if (mode == 2 && p == 7) index = 20;
      const auto& c = cases[index];
      device->law[p] = c.plastic ? fe::ShellSectionLaw::LayeredLaw44Nip3
          : (p % 2 ? fe::ShellSectionLaw::RigidSkin : fe::ShellSectionLaw::LayeredLaw1Nip3);
      device->policy[p] = c.policy;
      device->section[p] = c.section;
      std::memcpy(&device->value[p],&c.value,sizeof(c.value));
      device->active[p] = 19;
      valid[p] = c.value.policy() == c.policy && frozen_failure::ValidFailureEncoding(c.value) &&
          frozen_failure::ValidFailureState(c.value,c.policy,Section(c),time);
      if (!valid[p] && first == UINT32_MAX) first = p;
    }
    fe::qeph::batch_detail::LaunchMappedFailureActivity(&device->storage,KernelParents,0,time,
        &device->mixed,&device->failure,stream);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    EXPECT_EQ(device->first_invalid,first);
    for (unsigned p = 0; p < KernelParents; ++p) {
      if (valid[p]) EXPECT_EQ(device->active[p],device->value[p].active ? 1 : 0);
      else EXPECT_EQ(device->active[p],19u);
    }
  }
  ASSERT_EQ(cudaStreamDestroy(stream),cudaSuccess);
  ASSERT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace qeph_activity_test
