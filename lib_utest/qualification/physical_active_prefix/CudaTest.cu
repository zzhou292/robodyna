// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/elements/publication/PhysicalActivePrefix.h"
namespace physical_publication_test {
using Status=fe::ActivePrefixStatus;
bool Active(const fe::ActivePrefixReport& report) {
  EXPECT_EQ(report.status,Status::Ok)<<report.message;
  return report.status==Status::Ok;
}
TEST(PhysicalActivePrefix, ExactBudgetAndSourceAuthenticationAreFailureAtomic) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::PhysicalActivePrefix prefix;
  fe::ActivePrefixForecast forecast;
  ASSERT_TRUE(Active(fe::PhysicalActivePrefix::Preflight(rig.fixture.physical,{},forecast)));
  EXPECT_EQ(forecast.activity_capacity,2u);
  auto held=forecast;
  EXPECT_EQ(fe::PhysicalActivePrefix::Preflight(rig.fixture.physical,
      {forecast.owned_host_bytes-1},held).status,Status::ResourceLimit);
  EXPECT_EQ(held.owned_host_bytes,forecast.owned_host_bytes);
  auto* alias=reinterpret_cast<fe::ActivePrefixForecast*>(
      const_cast<fe::NodalCoefficientNode*>(rig.fixture.ledger.nodes().data()));
  EXPECT_EQ(fe::PhysicalActivePrefix::Preflight(rig.fixture.physical,{},*alias).status,Status::InvalidInput);
  auto incomplete=rig.Participants();incomplete.solids=nullptr;
  EXPECT_EQ(prefix.Initialize(rig.owner,rig.publication,rig.fixture.physical,
      incomplete,rig.fixture.Identity()).status,Status::SourceMismatch);
  EXPECT_EQ(prefix.allocations().owned_host_bytes,0u);
  EXPECT_EQ(prefix.Initialize(rig.owner,rig.publication,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity(),{forecast.owned_host_bytes-1}).status,Status::ResourceLimit);
  ASSERT_TRUE(Active(prefix.Initialize(rig.owner,rig.publication,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity(),{forecast.owned_host_bytes})));
  EXPECT_EQ(prefix.allocations().owned_host_bytes,forecast.owned_host_bytes);
  EXPECT_EQ(prefix.Initialize(rig.owner,rig.publication,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity()).status,Status::AlreadyInitialized);
  fe::FENodalState foreign;
  EXPECT_EQ(prefix.CheckAccepted(foreign,rig.publication).status,Status::SourceMismatch);
  ASSERT_TRUE(Active(prefix.CheckAccepted(rig.owner,rig.publication)));
}
TEST(PhysicalActivePrefix, GenuineCompleteCandidateRejectsForgedFieldsAndSurvivesDiscardCommit) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::PhysicalActivePrefix prefix;
  ASSERT_TRUE(Active(prefix.Initialize(rig.owner,rig.publication,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity())));
  Snapshot before,after;ASSERT_TRUE(rig.Read(before));
  fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  ASSERT_TRUE(Active(prefix.CheckPrepared(rig.owner,rig.publication,token,candidate,view)));
  auto forged=candidate;forged.qbat.active_count=0;
  EXPECT_EQ(prefix.CheckPrepared(rig.owner,rig.publication,token,forged,view).status,Status::SourceMismatch);
  rig.owner.Discard();rig.publication.DiscardTrial();
  ASSERT_TRUE(rig.Read(after));Exact(before,after);
  EXPECT_EQ(prefix.CheckPrepared(rig.owner,rig.publication,token,candidate,view).status,Status::SourceMismatch);
  ASSERT_TRUE(rig.Prepare(token,view,candidate));
  ASSERT_TRUE(Active(prefix.CheckPrepared(rig.owner,rig.publication,token,candidate,view)));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,token,candidate,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,Qualification,true})));
  ASSERT_TRUE(Active(prefix.CheckAccepted(rig.owner,rig.publication)));
  fe::PhysicalActivePrefix late;
  EXPECT_EQ(late.Initialize(rig.owner,rig.publication,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity()).status,Status::InvalidInput);
}
TEST(PhysicalActivePrefix, ActualMaterialRemovalIsRejectedBeforeCommonCommitAndExactlyRetries) {
  Rig rig(false,1e-9);
  ASSERT_TRUE(rig.Initialize());
  fe::PhysicalActivePrefix prefix;
  ASSERT_TRUE(Active(prefix.Initialize(rig.owner,rig.publication,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity())));
  Snapshot before,after;ASSERT_TRUE(rig.Read(before));
  for(unsigned retry=0;retry<2;++retry) {
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView view;
    fe::ShellPhysicalDiagnostics material,candidate;
    ASSERT_TRUE(rig.Begin(token,assembly));
    // Genuine nodal force drives the existing one-point material beyond its
    // declared failure strain. No history, activity or diagnostic is patched.
    const auto node=rig.fixture.domain.Find(14);
    const double force=2*rig.fixture.m[node]*.001/(H*H);
    ASSERT_EQ(cudaMemcpyAsync(assembly.forces.force_x+node,&force,sizeof(force),
        cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
    ASSERT_TRUE(rig.Advance(token,assembly,view));
    ASSERT_TRUE(rig.Evaluate(token,view,material));
    ASSERT_TRUE(Good(rig.publication.PreparePhysical(rig.owner,token,
        {&material.qeph,&material.t3,&material.qbat,&material.type25,&material.type13,&material.solids},&candidate)));
    const auto rejected=prefix.CheckPrepared(rig.owner,rig.publication,token,candidate,view);
    EXPECT_EQ(rejected.status,Status::InactiveParent)<<rejected.message;
    EXPECT_EQ(rejected.family,fe::ActivePrefixFamily::T3);
    EXPECT_EQ(rejected.parent,0u);
    ASSERT_TRUE(Active(prefix.CheckAccepted(rig.owner,rig.publication)));
    rig.owner.Discard();rig.publication.DiscardTrial();
    ASSERT_TRUE(rig.Read(after));Exact(before,after);
  }
}
} // namespace physical_publication_test
