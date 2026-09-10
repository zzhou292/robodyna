#include "PreparedSnapshotFixture.h"
namespace prepared_snapshot_test {
TEST_F(Cuda,EverySnapshotAndIdentityRangeRejectsOutputOrTokenAliasesBeforeChangingAnyBytes) {
  Fixture fixture;fe::FENodalState owner;ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));
  const auto token_bytes=Image(token);Output output;const auto before=output;
  for(unsigned i=0;i<6;++i) {
    for(unsigned j=0;j<i;++j) {
      auto buffer=output.nodes.buffer();Field(buffer,i)=Field(buffer,j);
      EXPECT_EQ(owner.CopyPrepared(token,buffer,&output.prepared).status,Code::InvalidInput);SameOutput(output,before);
    }
    auto buffer=output.nodes.buffer();Field(buffer,i)=reinterpret_cast<double*>(&output.prepared);
    EXPECT_EQ(owner.CopyPrepared(token,buffer,&output.prepared).status,Code::InvalidInput);SameOutput(output,before);
    buffer=output.nodes.buffer();Field(buffer,i)=reinterpret_cast<double*>(&token);
    EXPECT_EQ(owner.CopyPrepared(token,buffer,&output.prepared).status,Code::InvalidInput);
    SameOutput(output,before);EXPECT_EQ(Image(token),token_bytes);
  }
  EXPECT_EQ(owner.CopyPrepared(token,output.nodes.buffer(),reinterpret_cast<fe::NodalPreparedView*>(&token)).status,Code::InvalidInput);
  SameOutput(output,before);EXPECT_EQ(Image(token),token_bytes);ASSERT_TRUE(Copy(owner,token,output));
}
TEST_F(Cuda,SnapshotCapacityNullAndOverflowPreflightFailuresAllowSamePreparedTokenRetry) {
  Fixture fixture;fe::FENodalState owner;ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));
  const auto token_bytes=Image(token);Output output;const auto before=output;
  auto buffer=output.nodes.buffer();buffer.capacity_nodes=fixture.input.n-1;
  EXPECT_EQ(owner.CopyPrepared(token,buffer,&output.prepared).status,Code::ResourceLimit);SameOutput(output,before);
  for(unsigned i=0;i<6;++i) {
    buffer=output.nodes.buffer();Field(buffer,i)=reinterpret_cast<double*>(UINTPTR_MAX-4);
    EXPECT_EQ(owner.CopyPrepared(token,buffer,&output.prepared).status,Code::InvalidInput);SameOutput(output,before);
  }
  for(unsigned i=0;i<2;++i) {
    buffer=output.nodes.buffer();Field(buffer,i)=nullptr;
    EXPECT_EQ(owner.CopyPrepared(token,buffer,&output.prepared).status,Code::InvalidInput);SameOutput(output,before);
  }
  EXPECT_EQ(owner.CopyPrepared(token,output.nodes.buffer(),nullptr).status,Code::InvalidInput);SameOutput(output,before);
  EXPECT_EQ(Image(token),token_bytes);ASSERT_TRUE(Copy(owner,token,output));
}
TEST_F(Cuda,WrongStaleAndUnpreparedTokensCannotReadTrialOrAlterOutputAndAcceptedState) {
  Fixture fixture;fe::FENodalState owner,foreign,empty;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);ASSERT_EQ(fixture.Initialize(foreign).status,Code::Ok);
  nt::Snapshot accepted,held;ASSERT_TRUE(nt::Read(owner,accepted));
  fe::NodalTrialToken token,other;fe::NodalAssemblyView assembly,other_assembly;
  ASSERT_TRUE(Prepare(foreign,fixture.Load(),other,other_assembly));Output output;const auto before=output;
  EXPECT_EQ(empty.CopyPrepared(other,output.nodes.buffer(),&output.prepared).status,Code::NotInitialized);SameOutput(output,before);
  for(unsigned mode=0;mode<4;++mode) {
    ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));const auto stale=token;
    if(mode==1) {owner.Discard();ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));}
    if(mode==2) owner.Discard();
    if(mode==3) {owner.Discard();ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Code::Ok);}
    const auto& used=mode==0?other:(mode==1?stale:token);const auto token_bytes=Image(used);
    EXPECT_EQ(owner.CopyPrepared(used,output.nodes.buffer(),&output.prepared).status,mode<2?Code::StaleTrial:Code::WrongPhase);
    SameOutput(output,before);EXPECT_EQ(Image(used),token_bytes);
    ASSERT_TRUE(nt::Read(owner,held));nt::SameState(accepted,held);owner.Discard();
  }
}
TEST_F(Cuda,LastTrialReactionAndQuaternionFailuresPreserveAllOutputsAndAllowExactDiscardRetry) {
  Fixture fixture;fe::FENodalState owner,control;
  ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);ASSERT_EQ(fixture.Initialize(control).status,Code::Ok);
  for(unsigned failure=0;failure<2;++failure) {
    nt::Snapshot accepted,held;ASSERT_TRUE(nt::Read(owner,accepted));
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));
    fe::NodalPreparedView actual;ASSERT_EQ(owner.BorrowPrepared(token,&actual).status,Code::Ok);
    // Deliberate test-only corruption of the private packed slab. This is not a
    // supported application write path and creates no production fault hook.
    double bad=failure?2.:std::numeric_limits<double>::quiet_NaN();
    auto* target=const_cast<double*>(failure?actual.kinematics.orientation_wxyz+4*(fixture.input.n-1):
      actual.kinematics.position_xyz+19*fixture.input.n-1);
    ASSERT_EQ(cudaMemcpyAsync(target,&bad,sizeof bad,cudaMemcpyHostToDevice,actual.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(actual.stream),cudaSuccess);
    Output output;const auto before=output;const auto token_bytes=Image(token);
    EXPECT_EQ(owner.CopyPrepared(token,output.nodes.buffer(),&output.prepared).status,Code::InvalidOutput);
    SameOutput(output,before);EXPECT_EQ(Image(token),token_bytes);owner.Discard();
    ASSERT_TRUE(nt::Read(owner,held));nt::SameState(accepted,held);
    ASSERT_TRUE(Step(owner,fixture.Load()));ASSERT_TRUE(Step(control,fixture.Load()));SameOwners(owner,control);
  }
}
} // namespace prepared_snapshot_test
