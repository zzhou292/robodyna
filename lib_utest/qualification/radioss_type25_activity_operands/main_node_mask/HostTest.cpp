// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Native.h"
#include "lib_src/collision/radioss_type25/activity_operands/Values.h"
namespace activity_operands_test {
TEST(MainNodeRoleNative, ExactChkmsrPrefixRetiresOnlyZeroSupportAndNeverResurrects) {
  const int tags[]{1,0,1,0};int roles[]{1,2,-3,-4};
  robo_msr_retirement(4,4,tags,roles);
  EXPECT_EQ((std::vector<int>{roles,roles+4}),(std::vector<int>{1,-2,-3,-4}));
}
TEST(MainNodeRoleValues, KeepDisconnectedAffectsSecondaryRoleButNeverMainRetirement) {
  n::activity_source::Controls controls{n::activity_source::Deletion::ContainingElement,true,n::startup::SolidErosion::Disabled};
  EXPECT_EQ(a::detail::MainNodeActivity(true,false,controls),0);
  EXPECT_EQ(a::detail::MainNodeActivity(false,false,controls),1);
  EXPECT_EQ(a::detail::MarkSecondary(2.,false,controls),2.);
  controls.deletion=n::activity_source::Deletion::Disabled;
  EXPECT_EQ(a::detail::MainNodeActivity(true,false,controls),1);
}
}
