#include "T3BatchLedger.h"

namespace t3_batch_test {
TEST_F(T3BatchCuda, NativeScaleneStartupAndSharedMassBindOneTwoCellsWithBoundedStorage) {
  for(unsigned count:{1u,2u}) {
    SCOPED_TRACE(count); Rig r(count); ASSERT_TRUE(r.Initialize()); Results out{}; t::BatchDiagnostics d;
    const auto bytes=Bytes(out); const auto diagnostic=Bytes(d);
    EXPECT_EQ(r.batch.CopyAcceptedResults(r.owner.accepted(),out.data(),out.size(),&d).status,t::BatchStatus::NotBound);
    EXPECT_EQ(Bytes(out),bytes); EXPECT_EQ(Bytes(d),diagnostic);
    ASSERT_TRUE(r.Bind()); ASSERT_TRUE(Accepted(r,out,d));
    for(unsigned e=0;e<count;++e) {
      native::Reference reference; ASSERT_EQ(native::Initialize(oracle::Native(r.element[e].reference.input),reference),native::Status::kSuccess);
      oracle::StartupAgreement(r.element[e].reference,reference);
      EXPECT_GT(r.element[e].reference.input.node_ids[0],std::uint64_t{1}<<53);
      EXPECT_TRUE(out[e].proposed_history.matches_reference(r.element[e].reference));
      EXPECT_EQ(out[e].proposed_history.stamp().time,0); EXPECT_EQ(out[e].proposed_history.stamp().sample_index,0u);
      EXPECT_EQ(out[e].proposed_history.data().thickness,r.element[e].reference.input.thickness);
      EXPECT_EQ(out[e].proposed_history.data().active,1); EXPECT_FALSE(out[e].kinematics.valid);
      for(unsigned i=0;i<3;++i) for(unsigned a=0;a<3;++a) {
        EXPECT_EQ(oracle::Component(out[e].internal_force[i],a),0); EXPECT_EQ(oracle::Component(out[e].internal_couple[i],a),0);
      }
    }
    EXPECT_FALSE(d.has_completed_interval); EXPECT_EQ(d.minimum_native_dt,0); EXPECT_EQ(d.epoch,0u);
    const auto allocation=r.batch.allocations(); EXPECT_EQ(allocation.device_allocations,1u);
    EXPECT_GT(allocation.device_bytes,0u); EXPECT_LE(allocation.device_bytes,t::MaxBatchDeviceBytes);
    EXPECT_EQ(r.owner.allocations().device_allocations,6u);
    fe::NodalTrialToken token; fe::NodalPreparedView p; ASSERT_TRUE(Prepare(r,Loads{},token,p));
    ASSERT_TRUE(Candidate(r,p,d,out)); ASSERT_TRUE(Commit(r,token,d));
    EXPECT_EQ(r.batch.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(r.batch.allocations().device_allocations,allocation.device_allocations);
    EXPECT_EQ(r.owner.allocations().device_allocations,6u);
    EXPECT_TRUE(d.has_completed_interval); EXPECT_FALSE(d.accepted_force_assembled);
    EXPECT_EQ(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
    RecordProperty("batch_owned_device_bytes",std::to_string(allocation.device_bytes));
  }
}

TEST_F(T3BatchCuda, PrescribedLoadHoldReverseMatchesCompleteNativeHistoryAndIndependentWork) {
  unsigned comparisons=0;
  for(unsigned count:{1u,2u}) {
    Rig r(count); ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
    std::array<native::Reference,2> refs; std::array<native::History,2> histories;
    for(unsigned e=0;e<count;++e) {
      ASSERT_EQ(native::Initialize(oracle::Native(r.element[e].reference.input),refs[e]),native::Status::kSuccess);
      ASSERT_EQ(native::InitializeHistory(refs[e],{},histories[e]),native::Status::kSuccess);
    }
    Results accepted{}; t::BatchDiagnostics d; ASSERT_TRUE(Accepted(r,accepted,d));
    double maximum_membrane=0,maximum_bending=0,maximum_force=0;
    for(unsigned interval=0;interval<4;++interval) {
      SCOPED_TRACE(count);
      SCOPED_TRACE(interval);
      fe::NodalTrialToken token; fe::NodalPreparedView p; ASSERT_TRUE(Prepare(r,Schedule(r,interval),token,p));
      const auto endpoint=Endpoint(r,p); CheckPrescribedTargets(r,interval,endpoint); Results next{}; ASSERT_TRUE(Candidate(r,p,d,next));
      for(unsigned e=0;e<count;++e) {
        const auto in=Interval(r,e,p,endpoint); native::ForceTrial expected;
        ASSERT_EQ(native::EvaluateForce(refs[e],histories[e],oracle::Native(in),expected),native::Status::kSuccess);
        oracle::Agreement(r.element[e].reference,in,next[e],expected);
        oracle::oracle::Check(refs[e],histories[e].data(),oracle::Native(in),oracle::Native(refs[e],next[e]));
        oracle::Power(r.element[e].reference,in,next[e]);
        histories[e]=expected.proposed_history; ++comparisons;
        for(auto force:next[e].internal_force) maximum_force=std::max(maximum_force,std::abs(force.x)+std::abs(force.y)+std::abs(force.z));
      }
      CheckLedger(r,accepted,next,endpoint,d);
      maximum_membrane=std::max(maximum_membrane,std::abs(d.internal_work[0]));
      maximum_bending=std::max(maximum_bending,std::abs(d.internal_work[1]));
      EXPECT_EQ(d.kick_dt,interval?H:.5*H); EXPECT_EQ(d.velocity_time,p.velocity_time);
      ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(r,token,d));
      accepted=next; Results copied{}; t::BatchDiagnostics saved; ASSERT_TRUE(Accepted(r,copied,saved));
      EXPECT_EQ(saved.phase,t::BatchPhase::Accepted); EXPECT_EQ(saved.epoch,interval+1);
      for(unsigned e=0;e<count;++e) oracle::Exact(copied[e],next[e]);
    }
    EXPECT_GT(maximum_membrane,1e-12); EXPECT_GT(maximum_bending,1e-14); EXPECT_GT(maximum_force,1e-5);
  }
  RecordProperty("native_cell_intervals",comparisons);
  // GTest's numeric property overload is int; retain the exact frozen 1e-6 J
  // value as text instead of truncating reporting to zero. Budget unchanged.
  RecordProperty("aggregate_energy_scale_J","0.000001");
  RecordProperty("coupled_dynamics_qualified","false");
}

TEST_F(T3BatchCuda, AcceptedPositiveCacheUsesCompleteNativeNegativeSharedScatterWithoutHistoryAdvance) {
  Rig r; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  fe::NodalTrialToken token; fe::NodalPreparedView p; Results out{}; t::BatchDiagnostics d;
  ASSERT_TRUE(Prepare(r,Schedule(r,0),token,p)); ASSERT_TRUE(Candidate(r,p,d,out)); ASSERT_TRUE(Commit(r,token,d));
  ASSERT_TRUE(Accepted(r,out,d)); const auto accepted=Bytes(out); const auto diagnostic=Bytes(d);
  fe::NodalAssemblyView v; ASSERT_EQ(r.owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  Loads seed;
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    seed.force[3*n+a]=.125*(1+3*n+a); seed.couple[3*n+a]=-.0625*(1+3*n+a);
  }
  AddLoads<<<1,1,0,v.stream>>>(v,seed,1.);
  ASSERT_EQ(r.batch.AssembleAccepted(v).status,t::BatchStatus::Success);
  CheckNativeScatter(r,out,v);
  const auto once=Assembly(v); EXPECT_EQ(r.batch.AssembleAccepted(v).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(Assembly(v),once); EXPECT_EQ(r.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
  r.owner.Discard(); r.batch.DiscardTrial(); ASSERT_TRUE(Accepted(r,out,d));
  EXPECT_EQ(Bytes(out),accepted); EXPECT_EQ(Bytes(d),diagnostic);
}

TEST_F(T3BatchCuda, ImmutableUsageAndReceiptPreventUnadmittedCacheConsumption) {
  for(auto usage:{t::BatchUsage::PrescribedFields,t::BatchUsage::CoupledForces}) {
    Rig r(1); ASSERT_TRUE(r.Initialize(usage)); ASSERT_TRUE(r.Bind());
    fe::NodalTrialToken token; fe::NodalPreparedView p;
    ASSERT_TRUE(Prepare(r,Loads{},token,p,usage==t::BatchUsage::PrescribedFields));
    t::BatchDiagnostics d; const auto before=Bytes(d);
    EXPECT_EQ(r.batch.EvaluateCandidate(p,&d).status,t::BatchStatus::StaleTrial); EXPECT_EQ(Bytes(d),before);
    r.owner.Discard(); r.batch.DiscardTrial();
    // Coupled usage is tested ONLY at the exact zero rest state here. No
    // nonzero T3 force drives an owner interval in this TB1/TB2 gate.
    ASSERT_TRUE(Prepare(r,Loads{},token,p,usage==t::BatchUsage::CoupledForces));
    Results out{}; ASSERT_TRUE(Candidate(r,p,d,out));
    EXPECT_EQ(d.accepted_force_assembled,usage==t::BatchUsage::CoupledForces);
    EXPECT_EQ(r.owner.Commit(token).status,fe::NodalStatus::MissingCandidateValidation);
    EXPECT_NE(t::CommitT3Trial(r.owner,token,r.batch,d,Receipt(d)).status,t::BatchStatus::Success);
    EXPECT_EQ(r.owner.accepted().epoch,0u);
    ASSERT_TRUE(Prepare(r,Loads{},token,p,usage==t::BatchUsage::CoupledForces));
    ASSERT_TRUE(Candidate(r,p,d,out)); ASSERT_TRUE(Commit(r,token,d));
    EXPECT_EQ(r.owner.accepted().epoch,1u); EXPECT_EQ(d.kick_dt,.5*H);
  }
}
} // namespace t3_batch_test
