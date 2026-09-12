// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "lib_utest/qualification/extended_solid_resident/OwnerFixture.h"
#include "lib_src/solvers/ExplicitNodalRigidStep.h"

namespace law44_analytic_test {
namespace fe = tl::fea;
namespace solids = fe::solids;
using Peer = solids::BatchQualificationPeer;
using Results = extended_resident_test::Results;
namespace {
bool Good(fe::NodalReport r) { EXPECT_EQ(r.status, fe::NodalStatus::Ok) << r.message; return r.status == fe::NodalStatus::Ok; }
bool Good(solids::BatchReport r) { EXPECT_TRUE(r) << r.message; return bool(r); }
bool Prepare(extended_resident_test::OwnerFixture& fixture, fe::FENodalState& owner,
    const fe::NodalTrialToken& token, const fe::NodalAssemblyView& assembly, fe::NodalPreparedView& view) {
  fe::NodalCinAssemblyView cin;
  if (!Good(owner.BorrowCinAssembly(token, &cin))) return false;
  std::vector<double> stiffness(cin.node_count);
  if (cudaMemcpyAsync(stiffness.data(), cin.translational_stiffness, stiffness.size() * sizeof(double),
      cudaMemcpyDeviceToHost, cin.stream) != cudaSuccess || cudaStreamSynchronize(cin.stream) != cudaSuccess) return false;
  // The existing tiny owner fixture's disjoint CIN patch has its own declared contributor.
  const auto rows = fixture.Mechanics().cin_model.rows();
  for (std::size_t i = 0; i < rows.count; ++i) {
    for (auto node : rows.data[i].master_domain_nodes) stiffness[node] += 1;
    stiffness[rows.data[i].secondary_domain_node] += 1;
  }
  if (cudaMemcpyAsync(cin.translational_stiffness, stiffness.data(), stiffness.size() * sizeof(double),
      cudaMemcpyHostToDevice, cin.stream) != cudaSuccess ||
      cudaMemsetAsync(cin.witness_activity, 1, cin.witness_count, cin.stream) != cudaSuccess) return false;
  if (!Good(owner.SealAssembly(token)) || !Good(fe::AdvanceStaggeredCin(owner, token,
      {assembly.owner_id, assembly.accepted.base_epoch, assembly.attempt, cin.qualification_id,
       owner.accepted().fixed_dt, .2, true}))) return false;
  return Good(owner.BorrowPrepared(token, &view));
}
}
TEST(SolidLaw44AnalyticResident, MixedResidentMaterialReadbackLateFailureAndSoleOwnerRetry) {
  extended_resident_test::OwnerFixture fixture(true);
  auto& f = fixture.Mechanics();
  fe::FENodalState owner;
  auto config = f.Config(); config.fixed_dt = 1e-8;
  const auto cin = f.Cin();
  ASSERT_TRUE(Good(owner.Initialize(config, f.Kinematics(), f.im.data(), f.Dofs(), fixture.binding, &cin)));
  auto batch_config = fixture.Configuration(); batch_config.owner = owner.accepted();
  solids::Batch batch;
  ASSERT_TRUE(Good(batch.InitializeJoined(batch_config, fixture.model)));
  ASSERT_TRUE(Good(Peer::PreflightAttach(batch, owner, fixture.ledger, fixture.binding,
      fixture.Witnesses(), fixture.model, batch_config)));
  Peer::Attach(batch);
  Results base(fixture.model); solids::BatchDiagnostics accepted;
  ASSERT_TRUE(Good(batch.CopyAcceptedResults(owner.accepted(), base.Buffers(), &accepted)));
  EXPECT_EQ(accepted.epoch, 0u);
  for (const auto& point : base.rear[0].history.point) EXPECT_EQ(point.material.curve_cursor, 0u);
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    fe::NodalTrialToken token; fe::NodalAssemblyView assembly; fe::NodalPreparedView view;
    ASSERT_TRUE(Good(owner.BeginTrial(&token, &assembly)));
    ASSERT_TRUE(Good(batch.AssembleAccepted(owner, token, assembly)));
    ASSERT_TRUE(Prepare(fixture, owner, token, assembly, view));
    solids::BatchDiagnostics candidate;
    ASSERT_TRUE(Good(batch.EvaluateCandidate(owner, token, view, &candidate)));
    Results trial(fixture.model);
    ASSERT_TRUE(Good(batch.CopyPreparedResults(candidate, trial.Buffers())));
    for (const auto& point : trial.rear[0].history.point) EXPECT_EQ(point.material.curve_cursor, 0u);
    if (!attempt) {
      EXPECT_EQ(Peer::Commit(batch, owner, token, view, candidate, false).status, solids::BatchStatus::ElementFailure);
      EXPECT_EQ(owner.accepted().epoch, 0u);
      Results unchanged(fixture.model); solids::BatchDiagnostics old;
      ASSERT_TRUE(Good(batch.CopyAcceptedResults(owner.accepted(), unchanged.Buffers(), &old)));
      EXPECT_EQ(std::memcmp(&base.rear[0], &unchanged.rear[0], sizeof(base.rear[0])), 0);
    } else {
      ASSERT_TRUE(Good(Peer::Commit(batch, owner, token, view, candidate)));
      ASSERT_TRUE(Good(batch.CopyAcceptedResults(owner.accepted(), base.Buffers(), &accepted)));
      EXPECT_EQ(accepted.epoch, 1u);
    }
  }
}
} // namespace law44_analytic_test
