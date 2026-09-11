// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/solvers/cin_physical_mains/Values.h"
#include "../cin_physical_timestep/Fixture.h"
#include <gtest/gtest.h>
namespace cin_main_test {
namespace main=tl::fea::cin_physical_mains;
TEST(CinPhysicalMainsValues, ActualGroupCoefficientsAndCurrentScalarTermsIncludeZeroMassMembers) {
  cin_step_test::Fixture f;
  const auto view=f.View();
  main::Values first,second;
  std::uint32_t bad=UINT32_MAX;
  ASSERT_TRUE(main::Reduce(view.rigid,0,view.accepted,view.translation,view.rotation,view.nodes,first,bad));
  ASSERT_TRUE(main::Reduce(view.rigid,1,view.accepted,view.translation,view.rotation,view.nodes,second,bad));
  EXPECT_EQ(first.mass,5);EXPECT_EQ(first.minimum_inertia,2);
  EXPECT_EQ(first.translation,24);EXPECT_EQ(first.rotation,29);
  EXPECT_EQ(second.mass,3);EXPECT_EQ(second.minimum_inertia,.25);
  EXPECT_EQ(second.translation,300);EXPECT_EQ(second.rotation,705);
  EXPECT_EQ(second.center.x,2);EXPECT_EQ(second.center.y,.5);EXPECT_EQ(second.center.z,-.5);
  // The zero-M/J member contributes its actual stiffness, not a guessed mass.
  EXPECT_EQ(f.members[2].mass,0);EXPECT_EQ(f.members[2].inertia,0);
  f.members[2].mass=1e20;f.members[2].inertia=1e20;
  main::Values repeated;
  ASSERT_TRUE(main::Reduce(f.View().rigid,1,view.accepted,view.translation,view.rotation,view.nodes,repeated,bad));
  EXPECT_EQ(repeated.mass,second.mass);EXPECT_EQ(repeated.minimum_inertia,second.minimum_inertia);
  EXPECT_EQ(repeated.translation,second.translation);EXPECT_EQ(repeated.rotation,second.rotation);
  std::array<double,7> packet;
  main::Write(packet.data(),repeated);
  const auto restored=main::Read(packet.data());
  std::array<double,7> again;main::Write(again.data(),restored);
  EXPECT_EQ(packet,again);
}
TEST(CinPhysicalMainsValues, LastMemberAndGeometryFailurePreserveOutputThenExactRetry) {
  cin_step_test::Fixture f;
  auto view=f.View();
  main::Values result;
  std::uint32_t bad=UINT32_MAX;
  ASSERT_TRUE(main::Reduce(view.rigid,1,view.accepted,view.translation,view.rotation,view.nodes,result,bad));
  std::array<double,7> before,after;main::Write(before.data(),result);
  f.rotation[5]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(main::Reduce(view.rigid,1,view.accepted,view.translation,view.rotation,view.nodes,result,bad));
  EXPECT_EQ(bad,5u);main::Write(after.data(),result);EXPECT_EQ(before,after);
  f.rotation[5]=20;
  const auto original=f.accepted[15];f.accepted[15]=1e308;
  EXPECT_FALSE(main::Reduce(view.rigid,1,view.accepted,view.translation,view.rotation,view.nodes,result,bad));
  EXPECT_EQ(bad,5u);main::Write(after.data(),result);EXPECT_EQ(before,after);
  f.accepted[15]=original;
  ASSERT_TRUE(main::Reduce(view.rigid,1,view.accepted,view.translation,view.rotation,view.nodes,result,bad));
  main::Write(after.data(),result);EXPECT_EQ(before,after);
}
TEST(CinPhysicalMainsValues, ExactExistingScratchAndStagingBoundsPrecedeAccess) {
  EXPECT_TRUE(main::Fits(2,1,18,7));
  EXPECT_FALSE(main::Fits(2,1,17,7));
  EXPECT_FALSE(main::Fits(2,1,18,6));
  EXPECT_FALSE(main::Fits(2,2,18,14));
  EXPECT_FALSE(main::Fits(0,0,0,0));
  EXPECT_FALSE(main::Fits(SIZE_MAX,1,SIZE_MAX,SIZE_MAX));
  EXPECT_TRUE(main::Fits(372435,773,9*372435,19*372435));
  EXPECT_EQ(7*773*sizeof(double),43288u);
}
} // namespace cin_main_test
