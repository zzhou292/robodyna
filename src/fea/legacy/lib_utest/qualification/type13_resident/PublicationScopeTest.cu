// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace type13_resident_test {
using Peer = t::BatchQualificationPeer;

TEST_F(Type13ResidentCuda, PublicationAttachmentRequiresCompleteBoundSourceAndUniqueClaimant) {
  type13_model_test::Fixture input;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(input.Input(), true));
  const auto* claimant = Peer::Scope();
  const auto* other = Peer::Scope(1);
  const auto check = [&](const t::BatchConfig& config, const fe::Type13NodeContributions& source) {
    return Peer::PreflightAttach(rig.batch, config, source, claimant);
  };
  t::Batch unbound;
  EXPECT_EQ(Peer::PreflightAttach(unbound, rig.config, rig.source.contributions, claimant).status,
            t::BatchStatus::NotInitialized);
  ASSERT_TRUE(Good(unbound.InitializeJoined(rig.config, rig.source.contributions)));
  EXPECT_EQ(Peer::PreflightAttach(unbound, rig.config, rig.source.contributions, claimant).status,
            t::BatchStatus::NotBound);
  auto changed = rig.config;
  ++changed.configuration_id;
  EXPECT_EQ(check(changed, rig.source.contributions).status, t::BatchStatus::InvalidInput);
  changed = rig.config;
  ++changed.qualification_id;
  EXPECT_EQ(check(changed, rig.source.contributions).status, t::BatchStatus::InvalidInput);
  changed = rig.config;
  ++changed.owner.owner_id;
  EXPECT_EQ(check(changed, rig.source.contributions).status, t::BatchStatus::InvalidInput);
  changed = rig.config;
  changed.startup = {};
  EXPECT_EQ(check(changed, rig.source.contributions).status, t::BatchStatus::InvalidInput);
  changed = rig.config;
  changed.assembly = t::BatchAssembly::CinNativeStiffness;
  EXPECT_EQ(check(changed, rig.source.contributions).status, t::BatchStatus::InvalidInput);
  auto foreign_input = input.Input();
  ++foreign_input.source_instance_id;
  Source foreign;
  ASSERT_TRUE(foreign.Initialize(foreign_input));
  EXPECT_EQ(check(rig.config, foreign.contributions).status, t::BatchStatus::InvalidInput);
  EXPECT_EQ(Peer::PreflightAttach(rig.batch, rig.config, rig.source.contributions, nullptr).status,
            t::BatchStatus::InvalidInput);
  ASSERT_TRUE(Good(check(rig.config, rig.source.contributions)));
  Peer::Attach(rig.batch, claimant);
  EXPECT_EQ(check(rig.config, rig.source.contributions).status, t::BatchStatus::InvalidInput);
  Peer::Release(rig.batch, other);
  EXPECT_EQ(check(rig.config, rig.source.contributions).status, t::BatchStatus::InvalidInput);
  Peer::Release(rig.batch, claimant);
  ASSERT_TRUE(Good(check(rig.config, rig.source.contributions)));
}

TEST_F(Type13ResidentCuda, ForeignPublicationCannotAdvanceAndValidRetryPreservesNativeHistory) {
  type13_model_test::Fixture input;
  Rig rig;
  ASSERT_TRUE(rig.Initialize(input.Input()));
  ASSERT_TRUE(Good(Peer::PreflightAttach(rig.batch, rig.config, rig.source.contributions, Peer::Scope())));
  Peer::Attach(rig.batch, Peer::Scope());
  std::vector<t::Evaluation> initial;
  t::BatchDiagnostics initial_diagnostics;
  ASSERT_TRUE(rig.Read(initial, initial_diagnostics));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  fe::NodalPreparedView prepared;
  ASSERT_TRUE(rig.Begin(token, view, 1e9));
  ASSERT_TRUE(rig.Prepare(token, view, prepared));
  t::BatchDiagnostics candidate;
  ASSERT_TRUE(Good(rig.batch.EvaluateCandidate(rig.owner, token, prepared, &candidate)));
  EXPECT_EQ(Peer::Preflight(rig.batch, rig.owner, token, prepared, candidate, Peer::Scope(1)).status,
            t::BatchStatus::StaleTrial);
  EXPECT_EQ(Peer::Preflight(rig.batch, rig.owner, token, prepared, candidate, nullptr).status,
            t::BatchStatus::StaleTrial);
  EXPECT_EQ(rig.owner.accepted().epoch, 0u);
  std::vector<t::Evaluation> accepted;
  t::BatchDiagnostics unchanged;
  ASSERT_TRUE(rig.Read(accepted, unchanged));
  EXPECT_TRUE(detail::SameDiagnostics(initial_diagnostics, unchanged));
  for (std::size_t e = 0; e < accepted.size(); ++e) Exact(initial[e], accepted[e]);
  std::vector<t::Evaluation> values(initial.size()), native_next;
  ASSERT_TRUE(Good(rig.batch.CopyPreparedResults(candidate, values.data(), values.size())));
  rig.ComparePrepared(token, prepared, values, native_next);
  ASSERT_TRUE(Good(Peer::Commit(rig.batch, rig.owner, token, prepared, candidate)));
  EXPECT_EQ(rig.owner.accepted().epoch, 1u);
  Peer::Release(rig.batch, Peer::Scope());
  auto current = rig.config;
  current.owner = rig.owner.accepted();
  EXPECT_EQ(Peer::PreflightAttach(rig.batch, current, rig.source.contributions, Peer::Scope()).status,
            t::BatchStatus::InvalidInput);
  Peer::Poison(rig.batch);
  EXPECT_EQ(Peer::PreflightAttach(rig.batch, current, rig.source.contributions, Peer::Scope()).status,
            t::BatchStatus::Unusable);
  EXPECT_EQ(rig.batch.CopyAcceptedDiagnostics(rig.owner.accepted(), &unchanged).status,
            t::BatchStatus::Unusable);
}
} // namespace type13_resident_test
