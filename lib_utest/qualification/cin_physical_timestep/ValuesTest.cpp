// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
namespace cin_step_test {
TEST(CinPhysicalStepValues, NativeOrdinaryBoundaryZeroAndLateInvalidAreExplicit) {
  dt::ScalarLimit out;
  ASSERT_TRUE(dt::OrdinaryLimit(2,8,.8,out));
  EXPECT_DOUBLE_EQ(out.dt,.8*std::sqrt(.5));
  EXPECT_TRUE(out.bounded);
  const auto good=out;
  EXPECT_FALSE(dt::OrdinaryLimit(0,8,.8,out));
  EXPECT_EQ(std::memcmp(&good,&out,sizeof(out)),0);
  EXPECT_FALSE(dt::OrdinaryLimit(1,std::numeric_limits<double>::infinity(),.8,out));
  EXPECT_FALSE(dt::OrdinaryLimit(std::numeric_limits<double>::max(),1,.8,out));
  EXPECT_FALSE(dt::OrdinaryLimit(1,8,std::numeric_limits<double>::denorm_min(),out));
  EXPECT_EQ(std::memcmp(&good,&out,sizeof(out)),0);
  ASSERT_TRUE(dt::OrdinaryLimit(1,0,.8,out));
  EXPECT_FALSE(out.bounded);
  EXPECT_EQ(out.dt,std::numeric_limits<double>::max());
}
TEST(CinPhysicalStepValues, AllRolesAndTwoBodiesUseActualAggregateRatherThanMemberInverses) {
  Fixture f;
  dt::Result out;
  std::uint32_t bad=UINT32_MAX;
  const auto accepted=f.accepted;
  ASSERT_TRUE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.limiting_group,1u);
  EXPECT_EQ(out.limiting_node,4u);
  EXPECT_EQ(f.accepted,accepted);
  auto original=out;
  f.groups[0].mass=1e-9;
  ASSERT_TRUE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_EQ(out.limiting_group,0u);
  EXPECT_LT(out.minimum_dt,original.minimum_dt);
  f.groups[0].mass=5;
  f.mass[2]=1e20; f.inertia[2]=1e20;
  ASSERT_TRUE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_EQ(out.minimum_dt,original.minimum_dt);
  // Actual zero-M/J dependent member was already admitted; these coefficients
  // never define body response or its local stiffness trace.
  f.rotation[6]=1;
  auto saved=out;
  EXPECT_FALSE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_EQ(bad,6u);
  EXPECT_EQ(std::memcmp(&saved,&out,sizeof(out)),0);
  f.rotation[6]=0;
  f.groups[1].principal_inertia.z=0;
  EXPECT_FALSE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_EQ(bad,4u);
  EXPECT_EQ(std::memcmp(&saved,&out,sizeof(out)),0);
  f.groups[1].principal_inertia.z=1;
  ASSERT_TRUE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_EQ(out.minimum_dt,original.minimum_dt);
}
TEST(CinPhysicalStepValues, DisabledPolicyAndZeroStiffnessDoNotInventBoundOrRotation) {
  EXPECT_TRUE(fe::ValidCinStructuralStep({}));
  EXPECT_FALSE(fe::ValidCinStructuralStep({fe::NodalCinStructuralProfile::Disabled,.8}));
  EXPECT_FALSE(fe::ValidCinStructuralStep({fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,0}));
  EXPECT_FALSE(fe::ValidCinStructuralStep({static_cast<fe::NodalCinStructuralProfile>(9),.8}));
  Fixture f;
  f.translation.fill(0); f.rotation.fill(0);
  dt::Result out;
  std::uint32_t bad=UINT32_MAX;
  ASSERT_TRUE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_EQ(out.minimum_dt,std::numeric_limits<double>::max());
  EXPECT_EQ(out.limiting_node,UINT32_MAX);
  f.mass[6]=0;
  const auto saved=out;
  EXPECT_FALSE(dt::Screen(f.View(),.8,out,bad));
  EXPECT_EQ(bad,6u);
  EXPECT_EQ(std::memcmp(&saved,&out,sizeof(out)),0);
}
} // namespace cin_step_test
