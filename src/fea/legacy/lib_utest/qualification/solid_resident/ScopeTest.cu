// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace solid_resident_test {
TEST_F(SolidResidentCuda, InitialAuthorityClaimAndAssemblyBorrowRejectBeforeWrites) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize(false));
  const auto check=[&](const s::BatchConfig& config,const fe::NodalCinWitnessSource& cin,
      const fe::NodalCoefficientLedger& ledger) {
    return Peer::PreflightAttach(rig.batch,rig.owner,ledger,rig.fixture.mechanics.binding,
        cin,rig.fixture.model,config);
  };
  auto config=rig.config;
  ++config.configuration_id;
  EXPECT_EQ(check(config,rig.fixture.Witnesses(),rig.fixture.mechanics.ledger).status,s::BatchStatus::InvalidInput);
  auto cin=rig.fixture.Witnesses();
  --cin.witness_count;
  EXPECT_EQ(check(rig.config,cin,rig.fixture.mechanics.ledger).status,s::BatchStatus::InvalidInput);
  // Same declared shell domain with incomplete solid contributions is not this
  // model's initial source authority, even when node positions are identical.
  rigid_assembly_owner_test::Fixture incomplete;
  EXPECT_EQ(check(rig.config,rig.fixture.Witnesses(),incomplete.ledger).status,s::BatchStatus::InvalidInput);
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(Good(rig.owner.BeginTrial(&token,&assembly)));
  EXPECT_EQ(rig.batch.AssembleAccepted(rig.owner,token,assembly).status,s::BatchStatus::NotBound);
  rig.owner.Discard();
  ASSERT_TRUE(rig.Attach());
  EXPECT_EQ(check(rig.config,rig.fixture.Witnesses(),rig.fixture.mechanics.ledger).status,s::BatchStatus::InvalidInput);
  Peer::Release(rig.batch,Peer::Scope(1));
  Results initial;
  s::BatchDiagnostics diagnostics;
  ASSERT_TRUE(rig.Read(initial,diagnostics));
  ASSERT_TRUE(Good(rig.owner.BeginTrial(&token,&assembly)));
  const auto wrong_token=fe::NodalTrialToken{};
  EXPECT_FALSE(rig.batch.AssembleAccepted(rig.owner,wrong_token,assembly));
  // Failed Borrow discards the attempt and cannot leave a committable assembly.
  EXPECT_NE(rig.owner.SealAssembly(token).status,fe::NodalStatus::Ok);
  Results unchanged;
  ASSERT_TRUE(rig.Read(unchanged,diagnostics));
  Exact(initial,unchanged);
  ASSERT_TRUE(rig.Begin(token,assembly));
  rig.CompareAssembly(token,assembly,initial);
  rig.owner.Discard();rig.batch.DiscardTrial();
}
} // namespace solid_resident_test
