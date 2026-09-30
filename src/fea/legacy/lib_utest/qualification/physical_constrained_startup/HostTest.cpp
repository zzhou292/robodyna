// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/elements/ShellBatchStartup.h"
#include <gtest/gtest.h>
namespace fe=tl::fea;
namespace d=fe::shell_startup_detail;
TEST(ConstrainedStartupHost, FreeParticipantProofRetainsVelocityBitsAndLegacyScope) {
  const double q[]{1,0,0,0};const tl::math::Vec3 x{1,2,3};
  const fe::ShellBatchStartup input{fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation,{-0.,.5,-1.25}};
  EXPECT_FALSE(d::ValidStartup(input,true));EXPECT_TRUE(d::ValidStartup(input,true,true));
  EXPECT_FALSE(d::MatchesInitialNode(input,x,x,input.uniform_velocity,{},q));
  EXPECT_TRUE(d::MatchesInitialFreePhysicalNode(input,x,x,input.uniform_velocity,{},q));
  auto changed=input.uniform_velocity;changed.x=0.;
  EXPECT_FALSE(d::MatchesInitialFreePhysicalNode(input,x,x,changed,{},q));
  changed=input.uniform_velocity;changed.y=0.;
  EXPECT_FALSE(d::MatchesInitialFreePhysicalNode(input,x,x,changed,{},q));
  EXPECT_FALSE(d::MatchesInitialFreePhysicalNode(input,x,x,input.uniform_velocity,{0,0,1},q));
  for(const auto kind:{fe::ShellBatchStartupKind::ReferenceRest,fe::ShellBatchStartupKind::ReferenceUniformTranslation}) {
    const fe::ShellBatchStartup old{kind,kind==fe::ShellBatchStartupKind::ReferenceRest?tl::math::Vec3{}:input.uniform_velocity};
    EXPECT_EQ(d::MatchesInitialNode(old,x,x,old.uniform_velocity,{},q),
        d::MatchesInitialFreePhysicalNode(old,x,x,old.uniform_velocity,{},q));
  }
}
