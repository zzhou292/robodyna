// SPDX-License-Identifier: MIT
#include "FailureValuesFixture.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include <gtest/gtest.h>

namespace qeph_activity_test {
TEST(QephFailureActivityHost,CompleteFrozenFailureValuesAgreeAcrossPoliciesAndRawEncodings) {
  std::array<FailureCase,FailureCases> cases;
  FillFailureCases(cases);
  for (std::size_t i = 0; i < cases.size(); ++i) {
    SCOPED_TRACE(i);
    const auto& c = cases[i];
    EXPECT_EQ(SerialFailure(c),c.expected);
    EXPECT_EQ(fe::qeph::mapped::ValidFailureActivity(c.value,c.policy,Section(c),c.time),c.expected);
  }
  EXPECT_EQ(*reinterpret_cast<const unsigned char*>(&cases[18].value.active),2);
  EXPECT_EQ(*reinterpret_cast<const unsigned char*>(&cases[19].value.constant_points()[1].point_active),255);
  EXPECT_EQ(*reinterpret_cast<const unsigned char*>(&cases[20].value.tab1_points()[2].point_active),2);
}
TEST(QephFailureActivityHost,EveryPointAndStressChannelRemainsFreshlyValidated) {
  for (auto policy : {fe::ShellFailurePolicy::ConstantAllPoints,fe::ShellFailurePolicy::Tab1AnyPoint}) {
    for (unsigned p = 0; p < 3; ++p) {
      for (unsigned component = 0; component < 5; ++component) {
        fe::ShellBatchSectionState section;
        auto value = policy == fe::ShellFailurePolicy::ConstantAllPoints
            ? fe::ShellBatchFailureState::Constant() : fe::ShellBatchFailureState::Tab1();
        section.history.point[p].stress[component] = 8;
        value.current_force_point[p].stress[component] = 8;
        EXPECT_TRUE(fe::qeph::mapped::ValidFailureActivity(value,policy,&section,1));
        value.current_force_point[p].stress[component] = std::nextafter(8.,9.);
        EXPECT_FALSE(fe::qeph::mapped::ValidFailureActivity(value,policy,&section,1));
        value.current_force_point[p].stress[component] = 8;
        EXPECT_TRUE(fe::qeph::mapped::ValidFailureActivity(value,policy,&section,1));
      }
    }
  }
}
TEST(QephFailureActivityHost,FailureUsesExistingDevicePacketAndBoundedSeparateHostLifetime) {
  namespace q = fe::qeph;
  q::batch_detail::Layout legacy, mapped;
  ASSERT_TRUE(legacy.Initialize(324094,376930,fe::MaxVehicleShellResidentDeviceBytes));
  ASSERT_TRUE(mapped.InitializeMapped(324094,376930,fe::MaxVehicleShellResidentDeviceBytes));
  EXPECT_EQ(legacy.assembly.activity.bytes,0u);
  EXPECT_EQ(mapped.assembly.activity.active.count,324094u);
  EXPECT_EQ(mapped.bytes,mapped.assembly.activity.bytes);
  const auto exact = mapped.bytes;
  EXPECT_FALSE(mapped.InitializeMapped(324094,376930,exact - 1));
  EXPECT_EQ(mapped.bytes,exact);
  EXPECT_TRUE(mapped.InitializeMapped(324094,376930,exact));
  EXPECT_EQ(q::mapped::ActivityBytes(324094),324098u);
  using Buffer = tl::util::BoundedStartupArray<std::uint8_t,0>;
  EXPECT_EQ(Buffer::ExtraBytes(q::mapped::ActivityBytes(324094)),324162u);
  RecordProperty("additional_host_buffer_and_control_bytes","324162");
  RecordProperty("additional_device_bytes","0");
}
} // namespace qeph_activity_test
