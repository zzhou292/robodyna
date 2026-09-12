// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>
namespace beam18_publication_test {
TEST(BeamPublicationCuda, MissingAndForeignBeamCandidatesDiscardAllAndRetry) {
  Rig rig,foreign;
  ASSERT_TRUE(rig.Initialize()); ASSERT_TRUE(foreign.Initialize());
  Snapshot before,after; ASSERT_TRUE(rig.Read(before));
  for (unsigned attempt=0;attempt<2;++attempt) {
    fe::NodalTrialToken token; fe::NodalAssemblyView assembly; fe::NodalPreparedView view;
    fe::ShellPhysicalDiagnostics d,output;
    ASSERT_TRUE(rig.Begin(token,assembly)); ASSERT_TRUE(rig.mixed.Advance(token,assembly,view));
    ASSERT_TRUE(rig.Evaluate(token,view,d));
    auto candidates=Candidates(d);
    fe::ShellPhysicalDiagnostics foreign_candidate;
    if (!attempt) candidates.beam18=nullptr;
    else {
      fe::NodalTrialToken other_token; fe::NodalAssemblyView other_assembly; fe::NodalPreparedView other_view;
      ASSERT_TRUE(foreign.Begin(other_token,other_assembly));
      ASSERT_TRUE(foreign.mixed.Advance(other_token,other_assembly,other_view));
      ASSERT_TRUE(foreign.Evaluate(other_token,other_view,foreign_candidate));
      candidates.beam18=&foreign_candidate.beam18;
    }
    output.valid=true; const auto untouched=output;
    EXPECT_NE(rig.mixed.publication.PreparePhysical(rig.mixed.owner,token,candidates,&output).status,
        fe::ShellPublicationStatus::Success);
    EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(output,untouched));
    ASSERT_TRUE(rig.Read(after)); Exact(before,after);
    foreign.mixed.owner.Discard(); foreign.mixed.publication.DiscardTrial();
  }
  fe::NodalTrialToken token; fe::NodalPreparedView view; fe::ShellPhysicalDiagnostics d;
  ASSERT_TRUE(rig.Prepare(token,view,d));
  ASSERT_TRUE(Good(rig.mixed.publication.CommitPhysical(rig.mixed.owner,token,d,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,existing::Qualification,true})));
  EXPECT_EQ(rig.mixed.owner.accepted().epoch,1u);
}
TEST(BeamPublicationCuda, LateBeamFailureAndAlteredCompleteDiagnosticsPreserveEveryAcceptedField) {
  Rig rig; ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token; fe::NodalPreparedView view; fe::ShellPhysicalDiagnostics d;
  ASSERT_TRUE(rig.Prepare(token,view,d));
  ASSERT_TRUE(Good(rig.mixed.publication.CommitPhysical(rig.mixed.owner,token,d,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,existing::Qualification,true})));
  Snapshot before,after; ASSERT_TRUE(rig.Read(before));
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly)); ASSERT_TRUE(rig.mixed.Advance(token,assembly,view));
  ASSERT_TRUE(rig.Evaluate(token,view,d,false));
  const auto last=rig.source.model.parents()[2].domain_nodes[1];
  const double invalid=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.angular_velocity_xyz)+3*last+2,
      &invalid,sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  EXPECT_NE(rig.beam.EvaluateCandidate(rig.mixed.owner,token,view,&d.beam18).status,b::BatchStatus::Success);
  fe::ShellPhysicalDiagnostics output;
  EXPECT_NE(rig.mixed.publication.PreparePhysical(rig.mixed.owner,token,Candidates(d),&output).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);
  for (unsigned kind=0;kind<2;++kind) {
    ASSERT_TRUE(rig.Prepare(token,view,output));
    auto altered=output;
    if (!kind) altered.beam18.native_internal_work_increment_j[1]+=1;
    else altered.qeph.internal_drift_work+=1;
    EXPECT_NE(rig.mixed.publication.CommitPhysical(rig.mixed.owner,token,altered,
        {view.owner_id,view.kinematics.base_epoch,view.attempt,existing::Qualification,true}).status,
        fe::ShellPublicationStatus::Success);
    ASSERT_TRUE(rig.Read(after)); Exact(before,after);
  }
  ASSERT_TRUE(rig.Prepare(token,view,output));
  ASSERT_TRUE(Good(rig.mixed.publication.CommitPhysical(rig.mixed.owner,token,output,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,existing::Qualification,true})));
  EXPECT_EQ(rig.mixed.owner.accepted().epoch,2u);
}
} // namespace beam18_publication_test
