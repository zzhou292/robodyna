#include "GroupOwnerFixture.h"
namespace rigid_owner_test {
TEST_F(Cuda,FirstAndLaterValidationRejectionLeaveOneClockAndRetryExactly) {
  Fixture fixture; fe::FENodalState interrupted,control;
  ASSERT_EQ(fixture.Initialize(interrupted).status,Code::Ok); ASSERT_EQ(fixture.Initialize(control).status,Code::Ok);
  for(unsigned step=0;step<4;++step) {
    nt::Snapshot before,after; Groups group_before,group_after;
    ASSERT_TRUE(nt::Read(interrupted,before)); ASSERT_TRUE(Read(interrupted,group_before));
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_TRUE(Prepare(interrupted,fixture.Load(step),token,view));
    Groups trial; fe::NodalPreparedView prepared;
    ASSERT_EQ(interrupted.CopyPreparedRigidGroups(token,trial.buffer(),&prepared).status,Code::Ok);
    EXPECT_EQ(prepared.kick_dt,step?fixture.input.h:fixture.input.h/2);
    ASSERT_TRUE(Read(interrupted,group_after)); SameGroups(group_before,group_after);
    EXPECT_EQ(interrupted.Commit(token).status,Code::MissingCandidateValidation);
    interrupted.Discard();
    ASSERT_TRUE(Prepare(interrupted,fixture.Load(step),token,view));
    EXPECT_NE(fe::CompleteNodalValidation(interrupted,token,
      {view.owner_id,view.accepted.base_epoch,view.attempt,Qualification,false}).status,Code::Ok);
    interrupted.Discard();
    ASSERT_TRUE(nt::Read(interrupted,after)); nt::SameState(before,after);
    ASSERT_TRUE(Read(interrupted,group_after)); SameGroups(group_before,group_after);
    ASSERT_TRUE(Step(interrupted,fixture.Load(step))); ASSERT_TRUE(Step(control,fixture.Load(step)));
    SameOwners(interrupted,control);
  }
}

TEST_F(Cuda,LastMemberOfSecondGroupFailureRollsBackCompletedFirstGroupAndFreeNode) {
  Fixture fixture; fe::FENodalState interrupted,control;
  ASSERT_EQ(fixture.Initialize(interrupted).status,Code::Ok); ASSERT_EQ(fixture.Initialize(control).status,Code::Ok);
  for(unsigned epoch=0;epoch<2;++epoch) {
    nt::Snapshot before,after; Groups group_before,group_after;
    ASSERT_TRUE(nt::Read(interrupted,before)); ASSERT_TRUE(Read(interrupted,group_before));
    auto bad=fixture.Load();
    // Every node load is finite and sealable. The final source member overflows
    // the ordered group force sum after free-node and first-group trial writes.
    for(unsigned i=4;i<8;++i) bad.force[3*i]=i==7?6e307:4e307;
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_TRUE(nt::BeginLoad(interrupted,bad,token,view));
    ASSERT_EQ(interrupted.SealAssembly(token).status,Code::Ok);
    const auto report=fe::AdvanceStaggeredRigidGroups(interrupted,token,Admission(interrupted,view));
    EXPECT_EQ(report.status,Code::InvalidOutput); EXPECT_EQ(report.node,7u);
    EXPECT_NE(interrupted.Commit(token).status,Code::Ok); interrupted.Discard();
    ASSERT_TRUE(nt::Read(interrupted,after)); nt::SameState(before,after);
    ASSERT_TRUE(Read(interrupted,group_after)); SameGroups(group_before,group_after);
    ASSERT_TRUE(Step(interrupted,fixture.Load())); ASSERT_TRUE(Step(control,fixture.Load()));
    SameOwners(interrupted,control);
  }
}

TEST_F(Cuda,EveryOrdinaryAdvanceRejectsAttachedGroupsAndRigidPathRequiresQualification) {
  Fixture fixture; fe::FENodalState owner; ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  Groups before,after; ASSERT_TRUE(Read(owner,before));
  for(unsigned operation=0;operation<7;++operation) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_TRUE(nt::BeginLoad(owner,fixture.Load(),token,view)); ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
    fe::NodalReport report; auto admission=Admission(owner,view);
    switch(operation) {
      case 0: report=fe::AdvanceTranslations(owner,token); break;
      case 1: report=fe::AdvanceNodal(owner,token,{}); break;
      case 2: report=fe::AdvanceStaggeredPrescribed(owner,token,nt::Admission(owner,view)); break;
      case 3: report=fe::AdvanceStaggeredHistory(owner,token,admission); break;
      case 4: admission.qualification_id=0; report=fe::AdvanceStaggeredRigidGroups(owner,token,admission); break;
      case 5: admission.maximum_dt=fixture.input.h/2; report=fe::AdvanceStaggeredRigidGroups(owner,token,admission); break;
      case 6: ++admission.base_epoch; report=fe::AdvanceStaggeredRigidGroups(owner,token,admission); break;
    }
    EXPECT_EQ(report.status,operation==5?Code::StepTooLarge:operation==6?Code::StaleTrial:Code::MissingStepAdmission);
    owner.Discard(); ASSERT_TRUE(Read(owner,after)); SameGroups(before,after);
  }
  fe::FENodalState legacy; ASSERT_EQ(fixture.input.Initialize(legacy).status,Code::Ok);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_TRUE(nt::BeginLoad(legacy,fixture.Load(),token,view)); ASSERT_EQ(legacy.SealAssembly(token).status,Code::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredRigidGroups(legacy,token,Admission(legacy,view)).status,Code::MissingStepAdmission);
}

TEST_F(Cuda,NewSpinAdmissionRejectsWholeTrialAndReportsNoUnrelatedStableStep) {
  Fixture fixture; fe::FENodalState owner,control;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok); ASSERT_EQ(fixture.Initialize(control).status,Code::Ok);
  Groups before,after; ASSERT_TRUE(Read(owner,before));
  auto loads=fixture.Load(); loads.couple[0]=1e7;
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_TRUE(nt::BeginLoad(owner,loads,token,view)); ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  const auto report=fe::AdvanceStaggeredRigidGroups(owner,token,Admission(owner,view));
  EXPECT_EQ(report.status,Code::StepTooLarge); EXPECT_EQ(report.node,0u); EXPECT_EQ(report.stable_dt,0);
  owner.Discard(); ASSERT_TRUE(Read(owner,after)); SameGroups(before,after);
  SameOwners(owner,control);
  ASSERT_TRUE(Step(owner,fixture.Load())); ASSERT_TRUE(Step(control,fixture.Load())); SameOwners(owner,control);
}
} // namespace rigid_owner_test
