#include "WallCoupledFixture.h"
#include "WallCoupledResultChecks.h"

namespace qeph_wall_test {
namespace {
__global__ void CorruptEndpoint(double* x,unsigned i,double value) { x[i]=value; }
void ValidateStep(WallRig& w,const Snapshot& base,const Trial& t,const Staged& s,const PortResults& cache,
                  const NativeProposal& expected,LedgerEvidence& source,ContactEvidence& contact) {
  Identity(w,t,s); Agreement(w,t,s,expected,base.stamp.time,base.stamp.epoch);
  const auto host=w.Host(base,base.stamp.epoch,t.nodal.view.attempt);
  ContactAgreement(t.contact_base_result,host);
  CheckLedgers(w.shell,base,t.nodal,ContactLoads(host),cache,s.shell,source);
  CheckSourceWork(w.shell,cache,s.elements,s.shell,source); ContactLedgers(w,base,t,s,contact);
}
}
using WallCoupledCuda=tl_test::nodal_temporal::NodalTemporalCuda;
TEST_F(WallCoupledCuda, EitherParticipantFailureReadbackAndReceiptPreservePairThenRetryExactly) {
  for(unsigned failure=0;failure<7;++failure) {
    SCOPED_TRACE(failure);
    WallRig w(2); ASSERT_TRUE(w.Initialize(H0)); auto& r=w.shell;
    NativeSequence native; ASSERT_TRUE(native.Initialize(r));
    LedgerEvidence source; ContactEvidence contact;
    Snapshot initial; ASSERT_TRUE(Read(r.owner,initial));
    PortResults cache{}; q::BatchDiagnostics accepted; ASSERT_TRUE(ReadAccepted(r,cache,accepted));
    NativeProposal first; ASSERT_TRUE(native.Propose(r,ContactLoads(w.Host(native.state,0,1)),first));
    Trial first_trial; ASSERT_TRUE(Begin(w,first_trial));
    Staged first_stage; ASSERT_TRUE(Evaluate(w,first_trial,first_stage));
    ValidateStep(w,initial,first_trial,first_stage,cache,first,source,contact);
    sc::NodalWallDeviceResults published;
    ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Publish(w,first_trial,first_stage,published));
    native.Accept(r,first);
    Snapshot base; ASSERT_TRUE(Read(r.owner,base)); ASSERT_TRUE(ReadAccepted(r,cache,accepted));
    const auto cache_bytes=Bytes(cache); const auto accepted_bytes=Bytes(accepted); const auto published_bytes=Bytes(published);
    NativeProposal expected; ASSERT_TRUE(native.Propose(r,ContactLoads(w.Host(native.state,1,1)),expected));
    // A clean, fully checked inspect/discard attempt supplies exact retry truth.
    Trial clean; ASSERT_TRUE(Begin(w,clean)); Staged truth; ASSERT_TRUE(Evaluate(w,clean,truth));
    ValidateStep(w,base,clean,truth,cache,expected,source,contact);
    ASSERT_FALSE(::testing::Test::HasFailure()); w.Discard();
    Trial rejected; ASSERT_TRUE(Begin(w,rejected));
    Staged stage; stage.shell.owner_id=999; stage.contact.owner_id=998;
    if(failure==0) {
      ASSERT_EQ(w.wall.EvaluateCandidate(rejected.nodal.view,&stage.contact).status,sc::NodalWallDeviceStatus::Ok);
      ASSERT_EQ(w.wall.CopyResults(stage.contact,&stage.contact_result).status,sc::NodalWallDeviceStatus::Ok);
      const auto saved_shell=Bytes(stage.shell);
      // Last node belongs only to element 1. Corrupt TRIAL geometry after the
      // other participant has finished, never accepted owner storage/history.
      CorruptEndpoint<<<1,1,0,rejected.nodal.view.stream>>>(
        const_cast<double*>(rejected.nodal.view.kinematics.position_xyz),3*(r.n-1),
        std::numeric_limits<double>::quiet_NaN());
      ASSERT_EQ(cudaStreamSynchronize(rejected.nodal.view.stream),cudaSuccess);
      const auto failure_report=r.batch.EvaluateCandidate(rejected.nodal.view,&stage.shell);
      EXPECT_EQ(failure_report.status,q::BatchStatus::ElementFailure); EXPECT_EQ(failure_report.element,1u);
      EXPECT_EQ(Bytes(stage.shell),saved_shell);
    } else if(failure==1) {
      ASSERT_TRUE(Candidate(r,rejected.nodal.view,stage.shell,stage.elements));
      const auto saved_contact=Bytes(stage.contact);
      CorruptEndpoint<<<1,1,0,rejected.nodal.view.stream>>>(
        const_cast<double*>(rejected.nodal.view.kinematics.position_xyz),3*(r.n-1),2*DepthCap);
      ASSERT_EQ(cudaStreamSynchronize(rejected.nodal.view.stream),cudaSuccess);
      EXPECT_NE(w.wall.EvaluateCandidate(rejected.nodal.view,&stage.contact).status,sc::NodalWallDeviceStatus::Ok);
      EXPECT_EQ(Bytes(stage.contact),saved_contact);
    } else if(failure==2) {
      ASSERT_TRUE(Candidate(r,rejected.nodal.view,stage.shell,stage.elements));
      auto foreign=rejected.nodal.view; ++foreign.owner_id;
      const auto saved_contact=Bytes(stage.contact);
      EXPECT_NE(w.wall.EvaluateCandidate(foreign,&stage.contact).status,sc::NodalWallDeviceStatus::Ok);
      EXPECT_EQ(Bytes(stage.contact),saved_contact);
    } else {
      ASSERT_TRUE(Evaluate(w,rejected,stage));
      ValidateStep(w,base,rejected,stage,cache,expected,source,contact);
      if(failure==3 || failure==6) {
        auto stale=stage.contact;
        if(failure==3) ++stale.attempt; else ++stale.wall_binding_id;
        EXPECT_EQ(w.wall.CopyResults(stale,&published).status,sc::NodalWallDeviceStatus::StaleAttempt);
        EXPECT_EQ(Bytes(published),published_bytes);
      } else if(failure==4) {
        auto foreign=stage.shell; ++foreign.configuration_id;
        EXPECT_EQ(r.batch.CopyPreparedResults(foreign,cache.data(),cache.size()).status,q::BatchStatus::StaleTrial);
        EXPECT_EQ(Bytes(cache),cache_bytes);
      } else {
        auto receipt=Receipt(stage.shell); receipt.passed=false;
        EXPECT_EQ(q::CommitQephTrial(r.owner,rejected.nodal.token,r.batch,stage.shell,receipt).status,q::BatchStatus::StaleTrial);
      }
    }
    w.Discard();
    Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(base,after);
    ASSERT_TRUE(ReadAccepted(r,cache,accepted)); EXPECT_EQ(Bytes(cache),cache_bytes); EXPECT_EQ(Bytes(accepted),accepted_bytes);
    EXPECT_EQ(Bytes(published),published_bytes); EXPECT_EQ(native.epoch,1u); EXPECT_EQ(native.time,H0);
    EXPECT_EQ(w.wall.CopyResults(truth.contact,&published).status,sc::NodalWallDeviceStatus::StaleAttempt);
    EXPECT_EQ(Bytes(published),published_bytes);
    EXPECT_EQ(r.batch.CopyPreparedResults(truth.shell,cache.data(),cache.size()).status,q::BatchStatus::StaleTrial);
    EXPECT_EQ(Bytes(cache),cache_bytes);
    Trial retry; ASSERT_TRUE(Begin(w,retry)); EXPECT_NE(retry.nodal.view.attempt,rejected.nodal.view.attempt);
    EXPECT_EQ(retry.nodal.state.x,clean.nodal.state.x); EXPECT_EQ(retry.nodal.state.v,clean.nodal.state.v);
    EXPECT_EQ(retry.nodal.state.omega,clean.nodal.state.omega); EXPECT_EQ(retry.nodal.state.q,clean.nodal.state.q);
    Staged retried; ASSERT_TRUE(Evaluate(w,retry,retried,failure%2));
    EXPECT_EQ(Bytes(retried.elements),Bytes(truth.elements)); SameNumerics(retried.contact_result,truth.contact_result);
    ValidateStep(w,base,retry,retried,cache,expected,source,contact);
    ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Publish(w,retry,retried,published)); native.Accept(r,expected);
    ASSERT_TRUE(Read(r.owner,after)); OwnerAgreement(r,after,native.state); EXPECT_EQ(after.stamp.epoch,2u);
    ASSERT_TRUE(ReadAccepted(r,cache,accepted)); EXPECT_EQ(Bytes(cache),Bytes(truth.elements));
    EXPECT_EQ(published.diagnostics.attempt,retry.nodal.view.attempt);
  }
  RecordProperty("failure_variants",7); RecordProperty("maximum_accepted_intervals_per_owner",2);
  RecordProperty("native_cell_intervals",28); RecordProperty("combined_long_response_stability_qualified","false");
}
} // namespace qeph_wall_test
