// SPDX-License-Identifier: MIT
#include "../qeph_mapped_gather/Fixture.h"
#include <cuda_runtime.h>
#include <algorithm>

namespace qeph_activity_test {
namespace gather = qeph_gather_test;
namespace q = tl::fea::qeph;
namespace b = q::batch_detail;
TEST(QephMappedActivityCuda,CompleteFreshValidationRejectsRawActiveBeforeNarrowing) {
  gather::Fixture fixture;
  b::Storage* device = nullptr;
  gather::Inputs* input = nullptr;
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&device),fixture.layout.bytes),cudaSuccess);
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&input),sizeof(*input)),cudaSuccess);
  for (unsigned epoch : {0u,1u,2u}) {
    for (double raw : {0.,1.,257.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
      fixture.Prepare(epoch);
      auto& result = fixture.host->slab[0].element[1];
      auto& values = const_cast<q::HistoryValues&>(result.proposed_history.data());
      values.active = raw; // Deliberate raw device-cache fault; no material API admits it.
      if (!std::isfinite(raw) || raw < 0 || raw > 1) {
        // The last invalid parent must never hide the earlier malformed active field.
        fixture.host->slab[0].element[3].internal_couple[3].x = std::numeric_limits<double>::quiet_NaN();
      }
      std::uint32_t expected = UINT32_MAX;
      for (unsigned parent = 0; parent < gather::Parents; ++parent) {
        if (!q::mapped::ValidResult(fixture.host->model.element[parent].reference,
            fixture.host->slab[0].element[parent],epoch*1e-6,epoch,
            fixture.input.law[parent] == tl::fea::ShellSectionLaw::RigidSkin)) {
          expected = parent;
          break;
        }
      }
      std::memcpy(device,fixture.arena.data(),fixture.layout.bytes);
      *device = fixture.layout.Rebase(*fixture.host,device);
      *input = fixture.input;
      input->mixed.law = input->law;
      *device->assembly.activity.first_invalid = UINT32_MAX;
      std::fill_n(device->assembly.activity.active,gather::Parents,std::uint8_t(19));
      b::LaunchMappedActivity(device,gather::Parents,0,epoch*1e-6,epoch,&input->mixed,nullptr);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
      EXPECT_EQ(*device->assembly.activity.first_invalid,expected);
      if (expected == UINT32_MAX) EXPECT_EQ(device->assembly.activity.active[1],raw == 1 ? 1 : 0);
      else EXPECT_EQ(device->assembly.activity.active[1],19u);
    }
  }
  ASSERT_EQ(cudaFree(input),cudaSuccess);
  ASSERT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace qeph_activity_test
