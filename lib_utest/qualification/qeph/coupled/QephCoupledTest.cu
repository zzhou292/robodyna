#include "QephCoupledLedger.h"
#include <sstream>

namespace qeph_coupled_test {
using QephCoupledCuda=tl_test::nodal_temporal::NodalTemporalCuda;
namespace {
void Property(const std::string& name,double value) {
  std::ostringstream text; text<<std::setprecision(std::numeric_limits<double>::max_digits10)<<value;
  ::testing::Test::RecordProperty(name,text.str());
}
void CheckIdentity(const Rig& r,const Prepared& p,const q::BatchDiagnostics& d) {
  EXPECT_TRUE(d.valid); EXPECT_TRUE(d.has_completed_interval); EXPECT_TRUE(d.accepted_force_assembled);
  EXPECT_EQ(d.usage,q::BatchUsage::CoupledForces); EXPECT_EQ(d.phase,q::BatchPhase::Prepared);
  EXPECT_EQ(d.qualification_id,CoupledQualification); EXPECT_EQ(d.owner_id,r.owner.accepted().owner_id);
  EXPECT_EQ(d.base_epoch,r.owner.accepted().epoch); EXPECT_EQ(d.epoch,d.base_epoch+1);
  EXPECT_EQ(d.attempt,p.view.attempt); EXPECT_EQ(d.base_time,r.owner.accepted().time);
  EXPECT_EQ(d.time,p.view.proposed_time); EXPECT_EQ(d.velocity_time,p.view.velocity_time);
  EXPECT_EQ(d.base_velocity_time,p.view.base_velocity_time); EXPECT_EQ(d.kick_dt,p.view.kick_dt);
  EXPECT_EQ(d.kick_dt,d.base_epoch?r.h:.5*r.h);
}
void RunPrefix(unsigned cells) {
  for(unsigned refinement:{1u,2u}) {
    SCOPED_TRACE(cells);
    SCOPED_TRACE(refinement);
    Rig r(cells); ASSERT_TRUE(InitializeCoupled(r,H0/refinement));
    NativeSequence native; ASSERT_TRUE(native.Initialize(r));
    const auto allocation=r.batch.allocations(); const auto owner_allocation=r.owner.allocations();
    LedgerEvidence evidence; double startup_separation=0,feedback_separation=0;
    for(unsigned step=0;step<4*refinement;++step) {
      SCOPED_TRACE(step);
      Snapshot base; ASSERT_TRUE(Read(r.owner,base)); OwnerAgreement(r,base,native.state);
      EXPECT_EQ(base.stamp.epoch,native.epoch); EXPECT_EQ(base.stamp.time,native.time);
      PortResults cache{}; q::BatchDiagnostics accepted; ASSERT_TRUE(ReadAccepted(r,cache,accepted));
      const auto load=Applied(r,native.time); NativeProposal expected;
      ASSERT_TRUE(native.Propose(r,load,expected));
      Prepared p; ASSERT_TRUE(PrepareCoupled(r,load,p)); OwnerAgreement(r,p.state,expected.state);
      if(step==0) { startup_separation=WrongKickSeparation(r,base,p,load,true); EXPECT_GT(startup_separation,32.); }
      if(step==1) { feedback_separation=WrongKickSeparation(r,base,p,load,false); EXPECT_GT(feedback_separation,32.); }
      PortResults next{}; q::BatchDiagnostics d; ASSERT_TRUE(Candidate(r,p.view,d,next)); CheckIdentity(r,p,d);
      for(unsigned e=0;e<cells;++e) {
        SCOPED_TRACE(e);
        qeph_force_port_test::ForceAgreement(next[e],expected.cache[e],r.element[e].reference.input,
                                             Interval(r,e,p.state,native.time,native.epoch));
      }
      CheckLedgers(r,base,p,load,cache,d,evidence);
      CheckSourceWork(r,cache,next,d,evidence);
      ASSERT_FALSE(::testing::Test::HasFailure())<<"Never issue a receipt after a failed numerical check";
      const auto staged=Bytes(next); ASSERT_TRUE(Commit(r,p.token,d)); native.Accept(r,expected);
      Snapshot actual; ASSERT_TRUE(Read(r.owner,actual)); OwnerAgreement(r,actual,native.state);
      EXPECT_EQ(actual.x,p.state.x); EXPECT_EQ(actual.v,p.state.v); EXPECT_EQ(actual.omega,p.state.omega); EXPECT_EQ(actual.q,p.state.q);
      ASSERT_TRUE(ReadAccepted(r,next,accepted)); EXPECT_EQ(Bytes(next),staged);
      EXPECT_EQ(accepted.phase,q::BatchPhase::Accepted); EXPECT_EQ(accepted.epoch,native.epoch);
      EXPECT_EQ(accepted.attempt,d.attempt); EXPECT_EQ(accepted.time,native.time);
      // Reads neither advance native material nor allocate per interval.
      const auto output=Bytes(next); const auto diagnostics=Bytes(accepted);
      ASSERT_TRUE(ReadAccepted(r,next,accepted)); EXPECT_EQ(Bytes(next),output); EXPECT_EQ(Bytes(accepted),diagnostics);
      EXPECT_EQ(r.batch.allocations().device_bytes,allocation.device_bytes);
      EXPECT_EQ(r.batch.allocations().device_allocations,allocation.device_allocations);
      EXPECT_EQ(r.owner.allocations().device_bytes,owner_allocation.device_bytes);
      EXPECT_EQ(r.owner.allocations().device_allocations,owner_allocation.device_allocations);
    }
    const auto suffix="_refinement_"+std::to_string(refinement);
    EXPECT_EQ(r.owner.accepted().time,4*H0); EXPECT_EQ(r.owner.accepted().epoch,4*refinement);
    Property("startup_wrong_full_kick_budget_ratio"+suffix,startup_separation);
    Property("second_external_only_kick_budget_ratio"+suffix,feedback_separation);
    Property("maximum_kick_ledger_budget_ratio"+suffix,evidence.kick_ratio);
    Property("maximum_linear_momentum_budget_ratio"+suffix,evidence.momentum_ratio);
    Property("maximum_angular_momentum_budget_ratio"+suffix,evidence.angular_ratio);
    Property("maximum_internal_work_budget_ratio"+suffix,evidence.internal_work_ratio);
    Property("maximum_source_work_budget_ratio"+suffix,evidence.source_work_ratio);
    Property("maximum_stored_position_angular_rounding"+suffix,evidence.angular_drift_rounding);
    Property("final_native_source_work_diagnostic"+suffix,evidence.source_internal_work);
    Property("fixed_dt_s"+suffix,r.h);
  }
  ::testing::Test::RecordProperty("native_cell_intervals",12*cells);
  ::testing::Test::RecordProperty("long_trajectory_stability_qualified","false");
  ::testing::Test::RecordProperty("temporal_order_qualified","false");
}
}
TEST_F(QephCoupledCuda, OneCellRotaryFeedbackMatchesNativeAndClosesAllKickLedgers) {
  ASSERT_NO_FATAL_FAILURE(RunPrefix(1));
}
TEST_F(QephCoupledCuda, SharedTwoCellForceFeedbackMatchesNativeAndClosesAllKickLedgers) {
  ASSERT_NO_FATAL_FAILURE(RunPrefix(2));
}
TEST_F(QephCoupledCuda, RejectedNonzeroCoupledCandidatePreservesPairAndRetriesExactly) {
  Rig r(2); ASSERT_TRUE(InitializeCoupled(r,H0)); NativeSequence native; ASSERT_TRUE(native.Initialize(r));
  PortResults startup_cache{}; q::BatchDiagnostics startup_d;
  ASSERT_TRUE(ReadAccepted(r,startup_cache,startup_d));
  LedgerEvidence evidence;
  Prepared initial; NativeProposal first; const auto first_load=Applied(r,0);
  ASSERT_TRUE(native.Propose(r,first_load,first)); ASSERT_TRUE(PrepareCoupled(r,first_load,initial));
  PortResults initial_result{}; q::BatchDiagnostics first_d;
  ASSERT_TRUE(Candidate(r,initial.view,first_d,initial_result));
  for(unsigned e=0;e<r.count;++e) qeph_force_port_test::ForceAgreement(initial_result[e],first.cache[e],
    r.element[e].reference.input,Interval(r,e,initial.state,0,0));
  CheckSourceWork(r,startup_cache,initial_result,first_d,evidence);
  ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(r,initial.token,first_d)); native.Accept(r,first);
  Snapshot base; ASSERT_TRUE(Read(r.owner,base));
  PortResults accepted{}; q::BatchDiagnostics accepted_d; ASSERT_TRUE(ReadAccepted(r,accepted,accepted_d));
  const auto accepted_bytes=Bytes(accepted); const auto accepted_d_bytes=Bytes(accepted_d);
  double force=0; for(unsigned e=0;e<r.count;++e) for(auto f:accepted[e].internal_force) force+=qeph_startup_test::Length(f);
  ASSERT_GT(force,0.);
  const auto load=Applied(r,native.time); NativeProposal expected; ASSERT_TRUE(native.Propose(r,load,expected));
  Prepared rejected; ASSERT_TRUE(PrepareCoupled(r,load,rejected));
  PortResults candidate{}; q::BatchDiagnostics d; ASSERT_TRUE(Candidate(r,rejected.view,d,candidate)); CheckIdentity(r,rejected,d);
  CheckLedgers(r,base,rejected,load,accepted,d,evidence);
  CheckSourceWork(r,accepted,candidate,d,evidence);
  ASSERT_GT(WrongKickSeparation(r,base,rejected,load,false),32.);
  const auto candidate_bytes=Bytes(candidate); const auto diagnostic_bytes=Bytes(d);
  auto receipt=Receipt(d); receipt.passed=false;
  EXPECT_EQ(q::CommitQephTrial(r.owner,rejected.token,r.batch,d,receipt).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(candidate),candidate_bytes); EXPECT_EQ(Bytes(d),diagnostic_bytes);
  Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(base,after);
  ASSERT_TRUE(ReadAccepted(r,accepted,accepted_d)); EXPECT_EQ(Bytes(accepted),accepted_bytes); EXPECT_EQ(Bytes(accepted_d),accepted_d_bytes);
  EXPECT_EQ(native.epoch,1u); EXPECT_EQ(native.time,H0); // Native proposal has not been published either.
  EXPECT_EQ(r.batch.CopyPreparedResults(d,candidate.data(),candidate.size()).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(candidate),candidate_bytes);
  Prepared retry; ASSERT_TRUE(PrepareCoupled(r,load,retry)); EXPECT_NE(retry.view.attempt,rejected.view.attempt);
  EXPECT_EQ(retry.state.x,rejected.state.x); EXPECT_EQ(retry.state.v,rejected.state.v);
  EXPECT_EQ(retry.state.omega,rejected.state.omega); EXPECT_EQ(retry.state.q,rejected.state.q);
  PortResults retried{}; q::BatchDiagnostics rd; ASSERT_TRUE(Candidate(r,retry.view,rd,retried)); CheckIdentity(r,retry,rd);
  OwnerAgreement(r,retry.state,expected.state); CheckLedgers(r,base,retry,load,accepted,rd,evidence);
  CheckSourceWork(r,accepted,retried,rd,evidence);
  for(unsigned e=0;e<r.count;++e) {
    const auto interval=Interval(r,e,retry.state,H0,1);
    qeph_force_port_test::ForceAgreement(retried[e],candidate[e],r.element[e].reference.input,interval,0.);
    qeph_force_port_test::ForceAgreement(retried[e],expected.cache[e],r.element[e].reference.input,interval);
  }
  ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(r,retry.token,rd)); native.Accept(r,expected);
  ASSERT_TRUE(Read(r.owner,after)); OwnerAgreement(r,after,native.state); EXPECT_EQ(after.stamp.epoch,2u);
  ASSERT_TRUE(ReadAccepted(r,retried,accepted_d)); EXPECT_EQ(Bytes(retried),candidate_bytes);
  EXPECT_EQ(accepted_d.attempt,rd.attempt); EXPECT_EQ(accepted_d.epoch,2u);
  RecordProperty("native_cell_intervals",4); RecordProperty("cuda_cell_evaluations",6);
  RecordProperty("long_trajectory_stability_qualified","false");
}
} // namespace qeph_coupled_test
