// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include <limits>

namespace type45_resident_test {
namespace {
std::array<joint::Evaluation,3> JointSnapshot(Rig& rig) {
  std::array<joint::Result,3> result;joint::BatchDiagnostics diagnostics;
  EXPECT_TRUE(rig.CopyAccepted(result,diagnostics));
  std::array<joint::Evaluation,3> output;
  for(std::size_t j=0;j<3;++j) output[j]=joint::BatchQualificationPeer::Staged(rig.joints,j);
  return output;
}
void ExactJoints(const std::array<joint::Evaluation,3>& before,const std::array<joint::Evaluation,3>& after) {
  for(std::size_t j=0;j<3;++j) {
    EXPECT_TRUE(type45_test::EqualObservation(before[j],after[j]))<<j;
    if(before[j].history.ready()) EXPECT_TRUE(before[j].history.reference().Matches(after[j].history.reference()));
  }
}
}
TEST(Type45ResidentCuda,LateAttachAndOldInitializerRejectThenExactSevenSourceScope) {
  Rig rig;ASSERT_TRUE(rig.Initialize(joint::WorkingUnits::SI,false,false));
  const auto before=rig.physical.owner.accepted();
  auto& p=rig.physical;
  EXPECT_EQ(p.publication.InitializePhysical(p.owner,p.fixture.physical,p.fixture.rigid,
      p.fixture.WitnessSource(),rig.Participants(),p.fixture.Identity()).status,fe::ShellPublicationStatus::NotJoined);
  EXPECT_EQ(p.publication.InitializePhysicalWithJoints(p.owner,p.fixture.physical,p.fixture.rigid,
      p.fixture.WitnessSource(),rig.model,rig.Participants(),p.fixture.Identity()).status,
      fe::ShellPublicationStatus::NotInitialized);
  EXPECT_EQ(p.publication.allocations().device_bytes,0u);
  joint::BatchForecast forecast;ASSERT_TRUE(Good(joint::Batch::Forecast(rig.Config(),rig.model,forecast)));
  auto small=rig.Config();small.limits.max_device_bytes=forecast.device_bytes-1;
  EXPECT_EQ(rig.joints.InitializeJoined(small,rig.model).status,joint::BatchStatus::ResourceLimit);
  EXPECT_EQ(rig.joints.allocations().device_bytes,0u);
  auto exact=rig.Config();exact.limits.max_device_bytes=forecast.device_bytes;
  exact.limits.max_host_bytes=forecast.startup_host_bytes;
  ASSERT_TRUE(Good(rig.joints.InitializeJoined(exact,rig.model)));ASSERT_TRUE(rig.Attach());
  EXPECT_TRUE(fe::trial_identity::SameStamp(before,p.owner.accepted()));
  std::array<joint::Result,3> results;joint::BatchDiagnostics d;ASSERT_TRUE(rig.CopyAccepted(results,d));
  EXPECT_FALSE(d.automatic_stiffness_initialized);
  for(const auto& row:results) {
    EXPECT_FALSE(row.automatic_stiffness_initialized);EXPECT_EQ(row.context.target_dt_s,0);
    for(const auto& endpoint:row.endpoint) EXPECT_EQ(endpoint.translational_stiffness_n_m,0);
  }
  auto participants=rig.Participants();participants.type45=nullptr;
  EXPECT_EQ(p.publication.ValidatePhysicalSources(p.owner,p.fixture.physical,participants,p.fixture.Identity()).status,
      fe::ShellPublicationStatus::NotJoined);
  ASSERT_TRUE(Good(p.publication.ValidatePhysicalSources(p.owner,p.fixture.physical,rig.Participants(),p.fixture.Identity())));
  auto* overlap=reinterpret_cast<joint::Result*>(const_cast<joint::Joint*>(rig.model.joints().data()));
  EXPECT_EQ(rig.joints.CopyAcceptedResults(p.owner.accepted(),{overlap,3},&d).status,joint::BatchStatus::InvalidInput);
}
TEST(Type45ResidentCuda,ThreeKindsNativeRecurrenceAndExactRhsStiffnessScatterOnOneOwner) {
  for(auto units:{joint::WorkingUnits::SI,joint::WorkingUnits::MillimetreTonneSecond}) {
    SCOPED_TRACE(int(units));Rig rig;ASSERT_TRUE(rig.Initialize(units));
    auto& p=rig.physical;
    const auto allocation=rig.joints.allocations();
    std::vector<type45_test::NativeOracle> native;
    for(unsigned step=0;step<8;++step) {
      fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics candidates;
      ASSERT_TRUE(rig.Stage(token,view,candidates));
      if(!step) native=Native(rig,token,view);
      auto next_native=native;ComparePrepared(rig,view,candidates.type45,next_native);
      fe::ShellPhysicalDiagnostics ready;
      ASSERT_TRUE(Good(p.publication.PreparePhysical(p.owner,token,Candidates(candidates),&ready)));
      ASSERT_TRUE(Good(p.publication.CommitPhysical(p.owner,token,ready,
          {view.owner_id,view.kinematics.base_epoch,view.attempt,common::Qualification,true})));
      native=std::move(next_native); // Only a successful common acceptance advances native history.
      fe::ShellPhysicalDiagnostics accepted;
      ASSERT_TRUE(Good(p.publication.CopyAcceptedPhysicalDiagnostics(p.owner.accepted(),&accepted)));
      EXPECT_TRUE(accepted.has_type45);EXPECT_EQ(accepted.type45.epoch,step+1);
      EXPECT_EQ(accepted.type45.phase,joint::BatchPhase::Accepted);
      EXPECT_TRUE(accepted.type45.automatic_stiffness_initialized);
      EXPECT_EQ(accepted.type45.time,p.owner.accepted().time);
      EXPECT_EQ(rig.joints.allocations().device_bytes,allocation.device_bytes);
      EXPECT_EQ(rig.joints.allocations().device_allocations,allocation.device_allocations);
    }
  }
}
TEST(Type45ResidentCuda,LastJointFailureAndFalseCaptureKeepVirginAndLaterAcceptedHistories) {
  Rig rig;ASSERT_TRUE(rig.Initialize());auto& p=rig.physical;
  common::Snapshot before,after;ASSERT_TRUE(p.Read(before));
  auto before_joints=JointSnapshot(rig);
  std::array<joint::Result,3> results;joint::BatchDiagnostics initial;
  ASSERT_TRUE(rig.CopyAccepted(results,initial));
  fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics candidates;
  ASSERT_TRUE(rig.Stage(token,view,candidates,false));
  const auto last=rig.model.joints()[2].domain_nodes[1];
  const double invalid=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(view.kinematics.angular_velocity_xyz)+3*last+2,&invalid,
      sizeof(invalid),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  const auto failure=rig.joints.EvaluateCandidate(p.owner,token,view,&candidates.type45);
  EXPECT_EQ(failure.status,joint::BatchStatus::JointFailure);EXPECT_EQ(failure.joint,2u);
  fe::ShellPhysicalDiagnostics untouched;untouched.valid=true;const auto saved=untouched;
  EXPECT_NE(p.publication.PreparePhysical(p.owner,token,Candidates(candidates),&untouched).status,
      fe::ShellPublicationStatus::Success);
  EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(untouched,saved));
  ASSERT_TRUE(p.Read(after));common::Exact(before,after);
  ExactJoints(before_joints,JointSnapshot(rig));
  joint::BatchDiagnostics still_initial;ASSERT_TRUE(rig.CopyAccepted(results,still_initial));
  EXPECT_TRUE(joint::resident_detail::SameDiagnostics(initial,still_initial));
  for(const auto& row:results) EXPECT_FALSE(row.automatic_stiffness_initialized);
  ASSERT_TRUE(rig.Prepare(token,view,candidates));
  auto native=Native(rig,token,view);ComparePrepared(rig,view,candidates.type45,native);
  EXPECT_NE(p.publication.CommitPhysical(p.owner,token,candidates,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,common::Qualification,false}).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(p.Read(after));common::Exact(before,after);
  ExactJoints(before_joints,JointSnapshot(rig));
  ASSERT_TRUE(rig.CopyAccepted(results,still_initial));EXPECT_FALSE(still_initial.automatic_stiffness_initialized);
  ASSERT_TRUE(rig.Prepare(token,view,candidates));
  ASSERT_TRUE(Good(p.publication.CommitPhysical(p.owner,token,candidates,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,common::Qualification,true})));
  ASSERT_TRUE(p.Read(before));
  before_joints=JointSnapshot(rig);
  ASSERT_TRUE(rig.Prepare(token,view,candidates));
  auto forged=candidates;forged.type45.automatic_stiffness_initialized=false;
  EXPECT_NE(p.publication.CommitPhysical(p.owner,token,forged,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,common::Qualification,true}).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(p.Read(after));common::Exact(before,after);
  ExactJoints(before_joints,JointSnapshot(rig));
  ASSERT_TRUE(rig.Prepare(token,view,candidates));
  ASSERT_TRUE(Good(p.publication.CommitPhysical(p.owner,token,candidates,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,common::Qualification,true})));
  EXPECT_EQ(p.owner.accepted().epoch,2u);
}
} // namespace type45_resident_test
