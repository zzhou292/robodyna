// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_physical_activity/Storage.h"

#include <gtest/gtest.h>

#include <array>
#include <type_traits>

namespace self_contact_physical_activity_value_test {
namespace c = tlfea::contact;
namespace a = c::self_contact_physical_activity;

static_assert(!std::is_aggregate_v<c::SelfContactAcceptedActivityReceipt>);
static_assert(!std::is_aggregate_v<c::SelfContactPreparedActivityReceipt>);
static_assert(!std::is_copy_constructible_v<c::SelfContactPhysicalActivity>);

TEST(SelfContactPhysicalActivityValues,
     ExactArenaCapAndCompleteFamilyShapes) {
  a::Layout layout;
  ASSERT_TRUE(a::MakeLayout(4, 2, 1, 1, 12, layout));
  EXPECT_EQ(layout.accepted.count, 4u);
  EXPECT_EQ(layout.current.count, 4u);
  EXPECT_EQ(layout.qeph.count, 2u);
  EXPECT_EQ(layout.t3.count, 1u);
  EXPECT_EQ(layout.qbat.count, 1u);
  EXPECT_EQ(layout.bytes, 12u);
  EXPECT_FALSE(a::MakeLayout(4, 2, 1, 1, 11, layout));
  EXPECT_FALSE(a::MakeLayout(0, 2, 1, 1, 12, layout));
}

TEST(SelfContactPhysicalActivityValues,
     RemovalAndLongInactivePassButReactivationRejects) {
  const std::array<std::uint8_t, 4> base{{1, 1, 0, 0}};
  const std::array<std::uint8_t, 4> current{{1, 0, 0, 0}};
  EXPECT_EQ(a::ValidateTransition(
      base.data(), current.data(), base.size()).status,
      c::SelfContactPhysicalActivityStatus::Ok);

  auto reactivated = current;
  reactivated[2] = 1;
  const auto report = a::ValidateTransition(
      base.data(), reactivated.data(), base.size());
  EXPECT_EQ(report.status,
            c::SelfContactPhysicalActivityStatus::Reactivation);
  EXPECT_EQ(report.parent, 2u);

  auto invalid = current;
  invalid[3] = 2;
  EXPECT_EQ(a::ValidateTransition(
      base.data(), invalid.data(), base.size()).status,
      c::SelfContactPhysicalActivityStatus::InvalidActivity);
}

}  // namespace self_contact_physical_activity_value_test
