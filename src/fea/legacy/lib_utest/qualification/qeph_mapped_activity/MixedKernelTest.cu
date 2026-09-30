// SPDX-License-Identifier: MIT
#include "SerialMixedReadback.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/qeph/mapped/MixedActivityValues.h"
#include "lib_src/elements/ShellMixedSectionStorage.h"
#include <gtest/gtest.h>
#include <limits>
#include <new>

namespace qeph_activity_test {
namespace {
namespace fe = tl::fea;
namespace m = fe::qeph::mapped;
constexpr unsigned Count = 257;
struct MixedInput {
  fe::qeph::batch_detail::Storage storage;
  fe::shell_batch_plasticity_detail::MixedDeviceStorage mixed;
  fe::ShellSectionLaw law[Count];
  fe::ShellBatchSectionState plastic[Count];
  fe::sections::ShellLayeredLaw1History elastic[Count];
  std::uint32_t first_invalid;
  std::uint8_t roles[Count];
};
}
TEST(QephMixedActivityCuda,CompleteFrozenReaderAgreesWithRolesAndEarliestLateErrors) {
  MixedInput* device = nullptr;
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&device),sizeof(*device)),cudaSuccess);
  new(device) MixedInput{};
  cudaStream_t stream;
  ASSERT_EQ(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking),cudaSuccess);
  for (unsigned mode = 0; mode < 6; ++mode) {
    SCOPED_TRACE(mode);
    device->storage.model.config.element_count = Count;
    device->storage.assembly.activity = {&device->first_invalid,device->roles};
    device->mixed.law = device->law;
    device->mixed.plastic.section[0] = device->plastic;
    device->mixed.elastic_section[0] = device->elastic;
    device->first_invalid = UINT32_MAX;
    std::vector<fe::ShellBatchSectionState> plastic(Count);
    std::vector<fe::sections::ShellLayeredLaw1History> elastic(Count);
    frozen_mixed::ShellBatchPlasticityBinding catalog;
    catalog.roles.resize(Count);
    for (unsigned parent = 0; parent < Count; ++parent) {
      auto law = parent % 3 == 0 ? fe::ShellSectionLaw::LayeredLaw44Nip3 :
          (parent % 3 == 1 ? fe::ShellSectionLaw::LayeredLaw1Nip3 : fe::ShellSectionLaw::RigidSkin);
      // Corrupt unused capacity in every row: only the declared role is read.
      if (law != fe::ShellSectionLaw::LayeredLaw44Nip3)
        plastic[parent].cumulative_plastic_work_J = std::numeric_limits<double>::quiet_NaN();
      if (law != fe::ShellSectionLaw::LayeredLaw1Nip3)
        elastic[parent].point[2].stress[4] = std::numeric_limits<double>::quiet_NaN();
      if ((mode == 1 || mode == 2) && parent == Count-1)
        elastic[parent].point[0].stress[0] = std::numeric_limits<double>::infinity();
      if (mode == 2 && parent == 3)
        plastic[parent].history.point[1].filtered_rate_per_s = std::numeric_limits<double>::quiet_NaN();
      if (mode == 3 && parent == 5) law = fe::ShellSectionLaw::Law44Nip1;
      if (mode == 4 && parent == Count-1) law = static_cast<fe::ShellSectionLaw>(255);
      catalog.roles[parent] = device->law[parent] = law;
      device->plastic[parent] = plastic[parent];
      device->elastic[parent] = elastic[parent];
      device->roles[parent] = 19;
    }
    frozen_mixed::MixedHostStorage serial;
    serial.Bind(plastic,elastic);
    const auto expected = serial.Read(0,Count,nullptr,catalog);
    fe::qeph::batch_detail::LaunchMappedMixedActivity(&device->storage,Count,0,&device->mixed,stream);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    EXPECT_EQ(device->first_invalid == UINT32_MAX,
        expected.status == fe::shell_batch_plasticity_detail::SetupStatus::Success);
    unsigned first = UINT32_MAX;
    for (unsigned parent = 0; parent < Count; ++parent) {
      frozen_mixed::MixedHostStorage leaf;
      std::vector<fe::ShellBatchSectionState> p{plastic[parent]};
      std::vector<fe::sections::ShellLayeredLaw1History> e{elastic[parent]};
      leaf.Bind(p,e);
      frozen_mixed::ShellBatchPlasticityBinding source{{catalog.roles[parent]}};
      const auto report = leaf.Read(0,1,nullptr,source);
      if (report.status == fe::shell_batch_plasticity_detail::SetupStatus::Success)
        EXPECT_EQ(device->roles[parent],static_cast<std::uint8_t>(leaf.output_[0].law()));
      else {
        if (first == UINT32_MAX) first = parent;
        EXPECT_EQ(device->roles[parent],19u);
      }
    }
    if (first != UINT32_MAX) EXPECT_EQ(device->first_invalid/m::MixedActivityKeyStride,first);
  }
  ASSERT_EQ(cudaStreamDestroy(stream),cudaSuccess);
  ASSERT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace qeph_activity_test
