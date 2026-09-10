#include "TwoMemberOwnerFixture.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
namespace rigid_two_owner_test {
TEST_F(Cuda,TwoMemberFreeFlightPreservesReferenceArmsAndNativeStartupAssociation) {
  Fixture f;fe::FENodalState owner;
  const auto inverse=f.input.inverse[7];f.input.inverse[7]=2*inverse;
  EXPECT_EQ(f.Initialize(owner).status,Code::InvalidInput);f.input.inverse[7]=inverse;
  ASSERT_EQ(f.Initialize(owner).status,Code::Ok);
  namespace coefficients=fe::native_physical_coefficients;
  using Mass=tlfea::contact::TranslationMassModel;
  const auto scope=owner.rigid_groups();
  EXPECT_TRUE(coefficients::Admitted(scope,scope,Mass::kUnspecified,9));
  EXPECT_FALSE(coefficients::Admitted(scope,scope,Mass::kIsotropicLumped,9));
  EXPECT_FALSE(coefficients::ValidScope({781,4,7},9));
  EXPECT_FALSE(coefficients::Admitted({782,4,8},scope,Mass::kUnspecified,9));
  for(unsigned step=0;step<8;++step)ASSERT_TRUE(ro::Step(owner,nt::Loads{}));
  nt::Snapshot out;ASSERT_TRUE(nt::Read(owner,out));
  for(unsigned n=0;n<8;++n)for(unsigned a=0;a<3;++a) {
    rt::Agreement(out.x[3*n+a],f.input.x[3*n+a]+8*f.input.h*f.input.v[3*n+a]);
    EXPECT_EQ(out.v[3*n+a],f.input.v[3*n+a]);EXPECT_EQ(out.omega[3*n+a],0);
    EXPECT_EQ(out.reaction[3*n+a],0);EXPECT_EQ(out.couple[3*n+a],0);
  }
  fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(nt::BeginLoad(owner,f.Loads(),token,view));
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredPrescribed(owner,token,nt::Admission(owner,view)).status,Code::MissingStepAdmission);
  owner.Discard();
}
TEST_F(Cuda,TwoMemberLastGroupFailureAndRejectedValidationPreserveOneClockAndRetry) {
  Fixture f;fe::FENodalState owner,control;ASSERT_EQ(f.Initialize(owner,true).status,Code::Ok);
  ASSERT_EQ(f.Initialize(control,true).status,Code::Ok);
  for(unsigned epoch=0;epoch<3;++epoch) {
    nt::Snapshot before,after;Groups groups_before,groups_after;
    ASSERT_TRUE(nt::Read(owner,before));ASSERT_TRUE(Read(owner,groups_before));
    auto bad=f.Loads(epoch);bad.force[18]=bad.force[21]=1e308;
    fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(nt::BeginLoad(owner,bad,token,view));
    ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
    auto report=fe::AdvanceStaggeredRigidGroups(owner,token,ro::Admission(owner,view));
    EXPECT_EQ(report.status,Code::InvalidOutput);EXPECT_EQ(report.node,7u);
    EXPECT_NE(owner.Commit(token).status,Code::Ok);owner.Discard();
    ASSERT_TRUE(ro::Prepare(owner,f.Loads(epoch),token,view));
    EXPECT_EQ(owner.Commit(token).status,Code::MissingCandidateValidation);
    EXPECT_NE(fe::CompleteNodalValidation(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,ro::Qualification,false}).status,Code::Ok);
    owner.Discard();ASSERT_TRUE(nt::Read(owner,after));nt::SameState(before,after);
    ASSERT_TRUE(Read(owner,groups_after));SameGroups(groups_before,groups_after);
    ASSERT_TRUE(ro::Step(owner,f.Loads(epoch)));ASSERT_TRUE(ro::Step(control,f.Loads(epoch)));SameOwners(owner,control);
  }
}
TEST_F(Cuda,TwoMemberNewSpinDomainLimitKeepsAcceptedOutputAndExactRetry) {
  Fixture f;fe::FENodalState owner,control;ASSERT_EQ(f.Initialize(owner).status,Code::Ok);
  ASSERT_EQ(f.Initialize(control).status,Code::Ok);
  auto loads=f.Loads();loads.couple[21]=1;
  fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_TRUE(nt::BeginLoad(owner,loads,token,view));
  ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
  const auto report=fe::AdvanceStaggeredRigidGroups(owner,token,ro::Admission(owner,view));
  EXPECT_EQ(report.status,Code::StepTooLarge);EXPECT_EQ(report.stable_dt,0);owner.Discard();SameOwners(owner,control);
  ASSERT_TRUE(ro::Step(owner,f.Loads()));ASSERT_TRUE(ro::Step(control,f.Loads()));SameOwners(owner,control);
}
} // namespace rigid_two_owner_test
