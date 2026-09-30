// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../extended_solid_resident/OwnerFixture.h"
#include "lib_src/elements/ShellBatchStartup.h"
#include <gtest/gtest.h>
namespace constrained_extended_test {
namespace fe=tl::fea;
namespace s=fe::solids;
namespace x=extended_resident_test;
TEST(ConstrainedExtendedCuda, AllFiveFamiliesAuthenticateActualFreeSupportWithAbsentRotations) {
  x::OwnerFixture fixture;
  auto& f=fixture.Mechanics();
  const fe::ShellBatchStartup startup{fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation,{.125,.5,-1.25}};
  for(std::size_t node=0;node<f.domain.node_count();++node) {
    const auto v=fe::shell_startup_detail::ProjectVelocity(startup.uniform_velocity,f.fixed[node]);
    f.v[3*node]=v.x;f.v[3*node+1]=v.y;f.v[3*node+2]=v.z;
  }
  fe::FENodalState owner;
  const auto cin=f.Cin();auto config=f.Config();config.fixed_dt=1e-8;
  ASSERT_EQ(owner.Initialize(config,f.Kinematics(),f.im.data(),f.Dofs(),fixture.binding,&cin).status,
      fe::NodalStatus::Ok);
  auto batch_config=fixture.Configuration();batch_config.owner=owner.accepted();batch_config.startup=startup;
  s::Batch batch;
  ASSERT_TRUE(batch.InitializeJoined(batch_config,fixture.model));
  ASSERT_TRUE(s::BatchQualificationPeer::PreflightAttach(batch,owner,fixture.ledger,fixture.binding,
      fixture.Witnesses(),fixture.model,batch_config));
  // Existing qualification-only claim exposes constructed caches after the
  // real owner/ledger/CIN proof. No independent physical step is published.
  s::BatchQualificationPeer::Attach(batch);
  x::Results results(fixture.model);s::BatchDiagnostics d;
  ASSERT_TRUE(batch.CopyAcceptedResults(owner.accepted(),results.Buffers(),&d));
  EXPECT_FALSE(results.old18.empty());EXPECT_FALSE(results.old24.empty());EXPECT_FALSE(results.old6z.empty());
  EXPECT_FALSE(results.rear.empty());EXPECT_FALSE(results.foam.empty());
  EXPECT_EQ(owner.accepted().epoch,0u);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
  ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,fe::NodalStatus::Ok);
  ASSERT_TRUE(batch.AssembleAccepted(owner,token,assembly));
  owner.Discard();batch.DiscardTrial();
  EXPECT_EQ(owner.accepted().epoch,0u);
  s::BatchQualificationPeer::Release(batch);
}
} // namespace constrained_extended_test
