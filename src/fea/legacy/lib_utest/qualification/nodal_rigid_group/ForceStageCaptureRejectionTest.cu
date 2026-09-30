#include "ForceStageCaptureFixture.h"

namespace force_stage_capture_test {
namespace {
void* Range(Capture& out,unsigned i) {
  return i==0?static_cast<void*>(out.a.data()):i==1?static_cast<void*>(out.ar.data()):
    i==2?static_cast<void*>(out.groups.data()):static_cast<void*>(&out.prepared);
}
void Set(fe::NodalForceStageSnapshotBuffer& b,fe::NodalPreparedView*& p,unsigned i,void* value) {
  if(i==0)b.acceleration_xyz=static_cast<double*>(value);
  if(i==1)b.angular_acceleration_xyz=static_cast<double*>(value);
  if(i==2)b.groups=static_cast<fe::NodalRigidGroupAccelerationSnapshot*>(value);
  if(i==3)p=static_cast<fe::NodalPreparedView*>(value);
}
}
TEST_F(Cuda,AllForceStageOutputsRejectAliasesTokenRangesOverflowAndShortCapacitiesAtomically) {
  Fixture fixture;fe::FENodalState owner;ASSERT_EQ(Initialize(fixture,owner,Config(fixture)).status,Code::Ok);
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));
  const auto token_bytes=Image(token);Capture output;const auto untouched=output;
  for(unsigned i=0;i<4;++i) {
    for(unsigned j=0;j<i;++j) {
      auto b=output.buffer();auto* p=&output.prepared;Set(b,p,i,Range(output,j));
      EXPECT_EQ(owner.CopyPreparedForceStage(token,b,p).status,Code::InvalidInput);SameCapture(output,untouched);
    }
    for(void* invalid:{static_cast<void*>(&token),reinterpret_cast<void*>(UINTPTR_MAX-1),static_cast<void*>(nullptr)}) {
      auto b=output.buffer();auto* p=&output.prepared;Set(b,p,i,invalid);
      EXPECT_EQ(owner.CopyPreparedForceStage(token,b,p).status,Code::InvalidInput);SameCapture(output,untouched);
      EXPECT_EQ(Image(token),token_bytes);
    }
  }
  for(bool group:{false,true}) {
    auto b=output.buffer();if(group)b.capacity_groups=fixture.count-1;else b.capacity_nodes=fixture.input.n-1;
    EXPECT_EQ(owner.CopyPreparedForceStage(token,b,&output.prepared).status,Code::ResourceLimit);SameCapture(output,untouched);
  }
  ASSERT_EQ(owner.CopyPreparedForceStage(token,output.buffer(),&output.prepared).status,Code::Ok);
}
TEST_F(Cuda,WrongOwnerDiscardNewAttemptAndPreAdvanceStatesNeverExposeAnOldCapture) {
  Fixture fixture;fe::FENodalState owner,foreign,disabled,empty;
  ASSERT_EQ(Initialize(fixture,owner,Config(fixture)).status,Code::Ok);
  ASSERT_EQ(Initialize(fixture,foreign,Config(fixture)).status,Code::Ok);ASSERT_EQ(fixture.Initialize(disabled).status,Code::Ok);
  fe::NodalTrialToken token,other;fe::NodalAssemblyView assembly,other_view;
  ASSERT_TRUE(Prepare(foreign,fixture.Load(),other,other_view));Capture output;const auto untouched=output;
  EXPECT_EQ(empty.CopyPreparedForceStage(other,output.buffer(),&output.prepared).status,Code::NotInitialized);
  EXPECT_EQ(disabled.CopyPreparedForceStage(other,output.buffer(),&output.prepared).status,Code::WrongPhase);
  SameCapture(output,untouched);
  nt::Snapshot accepted,held;ASSERT_TRUE(nt::Read(owner,accepted));
  for(unsigned mode=0;mode<5;++mode) {
    ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));const auto stale=token;
    if(mode==1){owner.Discard();ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));}
    if(mode>=2)owner.Discard();
    if(mode>=3)ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,Code::Ok);
    if(mode==4)ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
    const auto& used=mode==0?other:mode==1?stale:token;const auto bytes=Image(used);
    EXPECT_EQ(owner.CopyPreparedForceStage(used,output.buffer(),&output.prepared).status,mode<2?Code::StaleTrial:Code::WrongPhase);
    SameCapture(output,untouched);EXPECT_EQ(Image(used),bytes);ASSERT_TRUE(nt::Read(owner,held));nt::SameState(accepted,held);owner.Discard();
  }
}
TEST_F(Cuda,LastNodeAndLastGroupNonfiniteScratchCannotPublishAndDiscardRetryIsExact) {
  Fixture fixture;fe::FENodalState owner,control;
  ASSERT_EQ(Initialize(fixture,owner,Config(fixture)).status,Code::Ok);
  ASSERT_EQ(Initialize(fixture,control,Config(fixture)).status,Code::Ok);
  for(bool group:{false,true}) {
    nt::Snapshot accepted,held;ASSERT_TRUE(nt::Read(owner,accepted));
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));
    const auto n=fixture.input.n,g=fixture.count;
    // Privileged test-only corruption of the private scratch tail. Applications
    // cannot write retained expired assembly pointers or this inferred layout.
    auto* target=assembly.forces.force_x+11*n+(group?6*(n+g):6*n)-1;
    const double invalid=std::numeric_limits<double>::quiet_NaN();
    ASSERT_EQ(cudaMemcpyAsync(target,&invalid,sizeof invalid,cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
    Capture out;const auto untouched=out;const auto bytes=Image(token);
    EXPECT_EQ(owner.CopyPreparedForceStage(token,out.buffer(),&out.prepared).status,Code::InvalidOutput);
    SameCapture(out,untouched);EXPECT_EQ(Image(token),bytes);owner.Discard();
    ASSERT_TRUE(nt::Read(owner,held));nt::SameState(accepted,held);
    fe::NodalTrialToken retry,reference;fe::NodalAssemblyView retry_view,reference_view;
    ASSERT_TRUE(Prepare(owner,fixture.Load(),retry,retry_view));ASSERT_TRUE(Prepare(control,fixture.Load(),reference,reference_view));
    Capture actual,expected;
    ASSERT_EQ(owner.CopyPreparedForceStage(retry,actual.buffer(),&actual.prepared).status,Code::Ok);
    ASSERT_EQ(control.CopyPreparedForceStage(reference,expected.buffer(),&expected.prepared).status,Code::Ok);
    EXPECT_EQ(actual.a,expected.a);EXPECT_EQ(actual.ar,expected.ar);EXPECT_EQ(Image(actual.groups),Image(expected.groups));
    ASSERT_TRUE(Accept(owner,retry,retry_view));ASSERT_TRUE(Accept(control,reference,reference_view));SameOwners(owner,control);
  }
}
} // namespace force_stage_capture_test
