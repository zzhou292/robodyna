// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/ShellPhysicalOwner.h"
#include "lib_src/solvers/NodalCinLayout.h"
#include <gtest/gtest.h>
#include "lib_src/constraints/tied_shell/runtime/CinForceStage.h"
namespace nodal_empty_test {
TEST(EmptyCinStartup, PreparedEmptyScopeKeepsExactDomainAndExplicitBackingIdentity) {
  Fixture f;f.Small();ASSERT_TRUE(f.cin.prepared());ASSERT_TRUE(f.cin.explicitly_empty());
  EXPECT_EQ(f.cin.rows().count,0u);EXPECT_EQ(f.cin.rows().data,nullptr);
  EXPECT_TRUE(f.cin.domain()->SharesStorage(f.domain));
  auto copy=f.cin;EXPECT_TRUE(copy.SharesStorage(f.cin));
  tied::TiedCinAttachmentModel separate;ASSERT_TRUE(tied::PrepareEmptyCinAttachments(f.domain,&separate));
  EXPECT_FALSE(separate.SharesStorage(f.cin));
  tied::CinAttachmentForecast forecast;
  ASSERT_TRUE(tied::ForecastEmptyCinAttachments(f.domain,0,&forecast));
  EXPECT_EQ(forecast.post_kinchk_payload_bytes,0u);EXPECT_EQ(forecast.scratch_bytes,0u);
  tied::TiedCinAttachmentModel fresh;
  auto limits=tied::CinAttachmentLimits{};limits.max_host_bytes=forecast.startup_payload_bytes;
  ASSERT_TRUE(tied::PrepareEmptyCinAttachments(f.domain,&fresh,limits));
  tied::TiedCinAttachmentModel short_cap;--limits.max_host_bytes;
  EXPECT_FALSE(tied::PrepareEmptyCinAttachments(f.domain,&short_cap,limits));EXPECT_FALSE(short_cap.prepared());
  tied::PostKinChkResult absent;
  EXPECT_FALSE(tied::PrepareCinAttachments(absent,f.domain,{nullptr,0},&separate));
  EXPECT_TRUE(separate.prepared());EXPECT_TRUE(separate.explicitly_empty());
}
TEST(EmptyCinStartup, EmptyLayoutIsExplicitAndRetainsPhysicalCoefficientsAndNodalScreen) {
  fe::nodal_detail::CinLayout layout;
  EXPECT_FALSE(layout.Initialize(7,0,0,{},128));
  ASSERT_TRUE(layout.Initialize(7,0,0,{},128,0,true));
  EXPECT_EQ(layout.state_values,4*7u+1);EXPECT_EQ(layout.scratch_values,9*7u);
  EXPECT_EQ(layout.rows.count,0u);EXPECT_EQ(layout.activity.count,0u);EXPECT_GT(layout.screen.count,0u);
  EXPECT_FALSE(layout.Initialize(7,1,0,{},128,0,true));
  EXPECT_FALSE(layout.Initialize(7,0,1,{},128,0,true));
  fe::shell_physical_owner::ProofLayout proof;
  ASSERT_TRUE(fe::shell_physical_owner::ForecastProof(7,0,1u<<20,proof));
  EXPECT_EQ(proof.coefficients.count,2*7u+1);
  const auto exact=proof.bytes;
  EXPECT_TRUE(fe::shell_physical_owner::ForecastProof(7,0,exact,proof));
  EXPECT_FALSE(fe::shell_physical_owner::ForecastProof(7,0,exact-1,proof));
}
TEST(EmptyCinStartup, ExplicitConstrainedProfileDoesNotWeakenLegacyProfiles) {
  const fe::ShellBatchStartup startup{fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation,{-0.,2.,-3.}};
  EXPECT_FALSE(fe::shell_startup_detail::ValidStartup(startup,true));
  EXPECT_TRUE(fe::shell_startup_detail::ValidStartup(startup,true,true));
  const double q[]{1,0,0,0};const tl::math::Vec3 x{1,2,3},spin{};
  for(std::uint8_t bits=0;bits<8;++bits) {
    const auto expected=fe::shell_startup_detail::ProjectVelocity(startup.uniform_velocity,bits);
    EXPECT_TRUE(fe::shell_startup_detail::MatchesConstrainedInitialNode(startup,bits,x,x,expected,spin,q));
    EXPECT_FALSE(fe::shell_startup_detail::MatchesInitialNode(startup,x,x,expected,spin,q));
    auto wrong=expected;wrong.z+=1;
    EXPECT_FALSE(fe::shell_startup_detail::MatchesConstrainedInitialNode(startup,bits,x,x,wrong,spin,q));
  }
  const auto partial=fe::shell_startup_detail::ProjectVelocity(startup.uniform_velocity,2);
  EXPECT_TRUE(fe::shell_startup_detail::SameBits(partial.x,-0.));
  EXPECT_TRUE(fe::shell_startup_detail::SameBits(partial.y,0.));
  EXPECT_FALSE(fe::shell_startup_detail::SameBits(partial.x,0.));
}
TEST(EmptyCinStartup, EmptyStageStillValidatesAllPhysicalNodesAndCopiesEntryInertia) {
  namespace c=tied::cin;
  const double x[]{0,0,0,1,0,0};double load[12]{},mass[]{2,3},inertia[]{.1,.2};
  double stiffness[]{4,5},rotation[]{6,7},numerical=0,entry[]{-1,-1};
  std::uint8_t dependent[]{0,0};c::StageView model{nullptr,dependent,2,0,0,nullptr,true};
  c::ForceTrial trial{x,load,mass,inertia,stiffness,rotation,nullptr,nullptr,&numerical,entry,nullptr,nullptr};
  ASSERT_TRUE(c::PrepareForceTrial(model,trial));EXPECT_EQ(entry[0],.1);EXPECT_EQ(entry[1],.2);
  EXPECT_EQ(mass[0],2);EXPECT_EQ(inertia[1],.2);EXPECT_EQ(numerical,0);
  model.explicitly_empty=false;EXPECT_FALSE(c::PrepareForceTrial(model,trial));
  model.explicitly_empty=true;dependent[1]=1;EXPECT_FALSE(c::PrepareForceTrial(model,trial));
  dependent[1]=0;stiffness[1]=-1;EXPECT_FALSE(c::PrepareForceTrial(model,trial));
  stiffness[1]=5;ASSERT_TRUE(c::PrepareForceTrial(model,trial));
}

}
