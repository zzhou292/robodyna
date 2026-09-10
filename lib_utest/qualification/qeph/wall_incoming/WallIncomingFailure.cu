#include "WallIncomingFixture.h"
#include "../wall_coupled/WallCoupledResultChecks.h"

namespace qeph_wall_incoming_test {
namespace {
__global__ void CorruptIncomingEndpoint(double* x,unsigned index,double value) { x[index]=value; }
}
void IncomingFailures(const ScreenBinding& binding) {
  IncomingRig w(2,binding); auto& r=w.coupled.shell;
  EXPECT_FALSE(w.Initialize(0)); EXPECT_FALSE(w.Initialize(3));
  EXPECT_EQ(r.owner.allocations().device_allocations,0u);
  EXPECT_EQ(r.batch.allocations().device_allocations,0u);
  EXPECT_EQ(w.coupled.wall.allocations().device_allocations,0u);
  ASSERT_TRUE(w.Initialize(1));
  const auto owner_allocation=r.owner.allocations();
  const auto shell_allocation=r.batch.allocations();
  const auto wall_allocation=w.coupled.wall.allocations();
  NativeSequence native; ASSERT_TRUE(w.InitializeNative(native));
  Snapshot initial; ASSERT_TRUE(Read(r.owner,initial));
  PortResults cache{}; q::BatchDiagnostics accepted; ASSERT_TRUE(ReadAccepted(r,cache,accepted));
  ASSERT_NO_FATAL_FAILURE(CheckInitial(w,initial,cache,accepted));
  IncomingEvidence evidence; sc::NodalWallDeviceResults published;
  ASSERT_GT(w.schedule.entry_base_epoch,1u);
  const auto crossing_base=w.schedule.entry_base_epoch-1;
  // Advance once to the prospective pre-entry state. Every rejection below
  // reuses this accepted state and the same independently proposed native pair.
  while(native.epoch<crossing_base) {
    SCOPED_TRACE(native.epoch);
    ASSERT_NO_FATAL_FAILURE(AdvanceIncoming(w,native,evidence,published));
    ASSERT_FALSE(::testing::Test::HasFailure());
  }
  ASSERT_EQ(evidence.crossing_base,UINT64_MAX);
  Snapshot base; ASSERT_TRUE(Read(r.owner,base)); ASSERT_TRUE(ReadAccepted(r,cache,accepted));
  ASSERT_EQ(base.stamp.epoch,crossing_base);
  for(unsigned n=0;n<r.n;++n) ASSERT_LT(base.x[3*n],0);
  EXPECT_NEAR(accepted.kinetic_translation,static_cast<double>(w.scales.energy),
              static_cast<double>((256*std::numeric_limits<double>::epsilon()+1e-12L)*w.scales.energy));
  const auto cache_bytes=Bytes(cache);
  const auto accepted_bytes=Bytes(accepted);
  const auto published_bytes=Bytes(published);
  const auto evidence_bytes=Bytes(evidence);
  const auto native_cache_bytes=Bytes(native.cache);
  const auto native_state=native.state;
  const auto native_time=native.time;
  const auto native_contact=w.coupled.Host(native.state,native.epoch,1);
  ASSERT_TRUE(native_contact.valid); ASSERT_EQ(native_contact.potential.upper,0);
  NativeProposal expected; ASSERT_TRUE(native.Propose(r,ContactLoads(native_contact),expected));
  Trial clean; ASSERT_TRUE(Begin(w.coupled,clean));
  Staged truth; ASSERT_TRUE(Evaluate(w.coupled,clean,truth));
  IncomingStage clean_stage;
  ASSERT_TRUE(CheckIncoming(w,base,clean,truth,cache,expected,evidence,clean_stage));
  ASSERT_TRUE(clean_stage.event.crossing); ASSERT_GT(clean_stage.premature_endpoint,32);
  ASSERT_FALSE(::testing::Test::HasFailure()); w.coupled.Discard();
  for(unsigned failure=0;failure<7;++failure) {
    SCOPED_TRACE(failure);
    Trial rejected; ASSERT_TRUE(Begin(w.coupled,rejected));
    Staged stage; stage.shell.owner_id=999; stage.contact.owner_id=998;
    if(failure==0) {
      ASSERT_EQ(w.coupled.wall.EvaluateCandidate(rejected.nodal.view,&stage.contact).status,sc::NodalWallDeviceStatus::Ok);
      ASSERT_EQ(w.coupled.wall.CopyResults(stage.contact,&stage.contact_result).status,sc::NodalWallDeviceStatus::Ok);
      const auto saved_shell=Bytes(stage.shell);
      // Last node belongs only to element one; contact has already succeeded.
      CorruptIncomingEndpoint<<<1,1,0,rejected.nodal.view.stream>>>(
        const_cast<double*>(rejected.nodal.view.kinematics.position_xyz),3*(r.n-1),
        std::numeric_limits<double>::quiet_NaN());
      ASSERT_EQ(cudaStreamSynchronize(rejected.nodal.view.stream),cudaSuccess);
      const auto report=r.batch.EvaluateCandidate(rejected.nodal.view,&stage.shell);
      EXPECT_EQ(report.status,q::BatchStatus::ElementFailure); EXPECT_EQ(report.element,1u);
      EXPECT_EQ(Bytes(stage.shell),saved_shell);
    } else if(failure==1) {
      ASSERT_TRUE(Candidate(r,rejected.nodal.view,stage.shell,stage.elements));
      const auto saved_contact=Bytes(stage.contact);
      CorruptIncomingEndpoint<<<1,1,0,rejected.nodal.view.stream>>>(
        const_cast<double*>(rejected.nodal.view.kinematics.position_xyz),3*(r.n-1),2*w.model.law().maximum_penetration);
      ASSERT_EQ(cudaStreamSynchronize(rejected.nodal.view.stream),cudaSuccess);
      EXPECT_NE(w.coupled.wall.EvaluateCandidate(rejected.nodal.view,&stage.contact).status,sc::NodalWallDeviceStatus::Ok);
      EXPECT_EQ(Bytes(stage.contact),saved_contact);
    } else if(failure==2) {
      ASSERT_TRUE(Candidate(r,rejected.nodal.view,stage.shell,stage.elements));
      auto foreign=rejected.nodal.view; ++foreign.owner_id;
      const auto saved_contact=Bytes(stage.contact);
      EXPECT_NE(w.coupled.wall.EvaluateCandidate(foreign,&stage.contact).status,sc::NodalWallDeviceStatus::Ok);
      EXPECT_EQ(Bytes(stage.contact),saved_contact);
    } else {
      ASSERT_TRUE(Evaluate(w.coupled,rejected,stage,failure%2));
      IncomingStage checked;
      ASSERT_TRUE(CheckIncoming(w,base,rejected,stage,cache,expected,evidence,checked));
      if(failure==3||failure==6) {
        auto stale=stage.contact;
        if(failure==3) ++stale.attempt; else ++stale.wall_binding_id;
        EXPECT_EQ(w.coupled.wall.CopyResults(stale,&published).status,sc::NodalWallDeviceStatus::StaleAttempt);
        EXPECT_EQ(Bytes(published),published_bytes);
      } else if(failure==4) {
        auto foreign=stage.shell; ++foreign.configuration_id;
        EXPECT_EQ(r.batch.CopyPreparedResults(foreign,cache.data(),cache.size()).status,q::BatchStatus::StaleTrial);
        EXPECT_EQ(Bytes(cache),cache_bytes);
      } else {
        auto receipt=Receipt(stage.shell); receipt.passed=false;
        EXPECT_EQ(q::CommitQephTrial(r.owner,rejected.nodal.token,r.batch,stage.shell,receipt).status,q::BatchStatus::StaleTrial);
        checked.checked=false;
        EXPECT_FALSE(PublishIncoming(w,rejected,checked,published,evidence));
      }
    }
    w.coupled.Discard();
    Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(base,after);
    ASSERT_TRUE(ReadAccepted(r,cache,accepted));
    EXPECT_EQ(Bytes(cache),cache_bytes); EXPECT_EQ(Bytes(accepted),accepted_bytes);
    EXPECT_EQ(Bytes(published),published_bytes); EXPECT_EQ(Bytes(evidence),evidence_bytes);
    SameState(native.state,native_state); EXPECT_EQ(Bytes(native.cache),native_cache_bytes);
    EXPECT_EQ(native.epoch,crossing_base); EXPECT_EQ(native.time,native_time);
    EXPECT_EQ(w.coupled.wall.CopyResults(truth.contact,&published).status,sc::NodalWallDeviceStatus::StaleAttempt);
    EXPECT_EQ(Bytes(published),published_bytes);
    EXPECT_EQ(r.batch.CopyPreparedResults(truth.shell,cache.data(),cache.size()).status,q::BatchStatus::StaleTrial);
    EXPECT_EQ(Bytes(cache),cache_bytes);
    // A clean inspect/discard retry after EVERY fault reproduces all physical
    // output, with only the authentic new attempt differing. No native rerun.
    Trial retry; ASSERT_TRUE(Begin(w.coupled,retry)); EXPECT_NE(retry.nodal.view.attempt,rejected.nodal.view.attempt);
    EXPECT_EQ(retry.nodal.state.x,clean.nodal.state.x); EXPECT_EQ(retry.nodal.state.v,clean.nodal.state.v);
    EXPECT_EQ(retry.nodal.state.omega,clean.nodal.state.omega); EXPECT_EQ(retry.nodal.state.q,clean.nodal.state.q);
    Staged retried; ASSERT_TRUE(Evaluate(w.coupled,retry,retried,failure%2));
    EXPECT_EQ(Bytes(retried.elements),Bytes(truth.elements)); SameNumerics(retried.contact_result,truth.contact_result);
    IncomingStage checked;
    ASSERT_TRUE(CheckIncoming(w,base,retry,retried,cache,expected,evidence,checked));
    ASSERT_TRUE(checked.event.crossing);
    w.coupled.Discard();
    EXPECT_EQ(Bytes(evidence),evidence_bytes); EXPECT_EQ(Bytes(published),published_bytes);
  }
  Trial retry; ASSERT_TRUE(Begin(w.coupled,retry));
  Staged retried; ASSERT_TRUE(Evaluate(w.coupled,retry,retried,true));
  EXPECT_EQ(Bytes(retried.elements),Bytes(truth.elements)); SameNumerics(retried.contact_result,truth.contact_result);
  IncomingStage checked;
  ASSERT_TRUE(CheckIncoming(w,base,retry,retried,cache,expected,evidence,checked));
  ASSERT_FALSE(::testing::Test::HasFailure());
  ASSERT_TRUE(PublishIncoming(w,retry,checked,published,evidence)); native.Accept(r,expected);
  EXPECT_EQ(evidence.crossing_base,crossing_base); EXPECT_EQ(evidence.first_applied_base,UINT64_MAX);
  // The first nonzero wall force enters this following kick, not the crossing
  // drift. Its omission must be independently resolvable above uncertainty.
  ASSERT_NO_FATAL_FAILURE(AdvanceIncoming(w,native,evidence,published));
  EXPECT_EQ(evidence.first_applied_base,crossing_base+1); EXPECT_GT(evidence.omitted_contact,32);
  EXPECT_EQ(r.owner.allocations().device_bytes,owner_allocation.device_bytes);
  EXPECT_EQ(r.owner.allocations().device_allocations,owner_allocation.device_allocations);
  EXPECT_EQ(r.batch.allocations().device_bytes,shell_allocation.device_bytes);
  EXPECT_EQ(r.batch.allocations().device_allocations,shell_allocation.device_allocations);
  EXPECT_EQ(w.coupled.wall.allocations().device_bytes,wall_allocation.device_bytes);
  EXPECT_EQ(w.coupled.wall.allocations().device_allocations,wall_allocation.device_allocations);
  RecordIncoming(w,evidence,"_failure_retry");
  ::testing::Test::RecordProperty("screen_decision_sha256",binding.decision_sha256);
  ::testing::Test::RecordProperty("failure_variants",7);
  ::testing::Test::RecordProperty("native_cell_intervals",static_cast<int>(2*native.epoch));
  ::testing::Test::RecordProperty("full_response_qualified","false");
}
} // namespace qeph_wall_incoming_test
