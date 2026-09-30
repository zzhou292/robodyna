#include "QephBatchFixture.h"
#include "lib_utest/qualification/native/qeph/NativeQephBridge.h"
extern "C" void qeph_q2_scatter(const double*,const double*,const int*,double*);

namespace qeph_batch_test {
TEST_F(QephBatchCuda, StartupBindsActualRestAndRetainsBoundedAllocationAcrossOneTwoFourCells) {
  for(unsigned count:{1u,2u,4u}) for(bool disjoint:{false,true}) {
    SCOPED_TRACE(count);
    SCOPED_TRACE(disjoint);
    Rig rig(count,false,disjoint); ASSERT_TRUE(rig.Initialize());
    std::array<q::ForceTrial,4> result{}; q::BatchDiagnostics d;
    const auto untouched=Bytes(result); const auto diagnostic=Bytes(d);
    EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(),result.data(),4,&d).status,q::BatchStatus::NotBound);
    EXPECT_EQ(Bytes(result),untouched); EXPECT_EQ(Bytes(d),diagnostic);
    ASSERT_TRUE(rig.Bind()); ASSERT_TRUE(ReadAccepted(rig,result,d));
    EXPECT_FALSE(d.has_completed_interval); EXPECT_EQ(d.epoch,0u); EXPECT_EQ(d.phase,q::BatchPhase::Accepted);
    for(unsigned e=0;e<count;++e) {
      EXPECT_TRUE(result[e].proposed_history.matches_reference(rig.element[e].reference));
      EXPECT_EQ(result[e].proposed_history.stamp().sample_index,0u);
      EXPECT_EQ(result[e].proposed_history.data().thickness,rig.element[e].reference.input.thickness);
      for(auto f:result[e].internal_force) EXPECT_EQ(qeph_startup_test::Length(f),0.);
    }
    const auto allocation=rig.batch.allocations(); EXPECT_EQ(allocation.device_allocations,1u);
    EXPECT_GT(allocation.device_bytes,0u); EXPECT_LE(allocation.device_bytes,q::MaxBatchDeviceBytes);
    EXPECT_EQ(rig.owner.allocations().device_allocations,6u);
    fe::NodalTrialToken token; fe::NodalPreparedView p; ASSERT_TRUE(Prepare(rig,Loads{},token,p));
    ASSERT_TRUE(Candidate(rig,p,d,result)); ASSERT_TRUE(Commit(rig,token,d));
    EXPECT_EQ(rig.batch.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(rig.batch.allocations().device_allocations,allocation.device_allocations);
    ASSERT_TRUE(ReadAccepted(rig,result,d)); EXPECT_EQ(d.epoch,1u); EXPECT_TRUE(d.has_completed_interval);
    EXPECT_FALSE(d.accepted_force_assembled); EXPECT_EQ(d.kinetic_translation,0.); EXPECT_EQ(d.kinetic_rotation,0.);
    RecordProperty("batch_owned_device_bytes",std::to_string(allocation.device_bytes));
  }
  RecordProperty("maximum_batch_nodes",16); RecordProperty("dynamics_qualified","false");
}

TEST_F(QephBatchCuda, PrescribedSharedNodeSequenceMatchesEveryNativeFieldAndGlobalKineticLedger) {
  using qeph_force_port_test::ForceAgreement;
  for(bool warped:{false,true}) {
    Rig rig(4,warped); ASSERT_TRUE(rig.Initialize()); ASSERT_TRUE(rig.Bind());
    std::array<native::Reference,4> refs; std::array<native::History,4> histories;
    for(unsigned e=0;e<4;++e) {
      ASSERT_EQ(native::Initialize(qeph_startup_test::NativeInput(rig.element[e].reference.input),refs[e]),native::Status::kSuccess);
      ASSERT_EQ(native::InitializeHistory(refs[e],{},histories[e]),native::Status::kSuccess);
    }
    for(double sign:{1.,1.,0.,-1.,-1.}) {
      const auto load=Pattern(rig,sign); fe::NodalTrialToken token; fe::NodalPreparedView p;
      ASSERT_TRUE(Prepare(rig,load,token,p));
      q::BatchDiagnostics d; std::array<q::ForceTrial,4> out; ASSERT_TRUE(Candidate(rig,p,d,out));
      long double kt=0,kj=0,kp=0,ka=0;
      std::array<bool,N> seen{};
      std::array<long double,N> mass{},inertia{},physical{},added{};
      std::array<q::Vec3,N> velocity{},omega{};
      for(unsigned e=0;e<4;++e) {
        const auto interval=ReadInterval(rig,e,p); native::ForceTrial truth;
        ASSERT_EQ(native::EvaluateForce(refs[e],histories[e],qeph_kinematics_test::NativeInterval(interval),truth),native::Status::kSuccess);
        ForceAgreement(out[e],truth,rig.element[e].reference.input,interval);
        histories[e]=truth.proposed_history;
        // Independent projected diagonal area (not true warped surface area).
        const auto& in=rig.element[e].reference.input;
        const auto cross=qeph_kinematics_test::Cross(qeph_startup_test::Difference(in.position[2],in.position[0]),
                                                qeph_startup_test::Difference(in.position[3],in.position[1]));
        const long double area=.5L*std::sqrt(static_cast<long double>(cross.x)*cross.x+
          static_cast<long double>(cross.y)*cross.y+static_cast<long double>(cross.z)*cross.z);
        const long double m=in.density*static_cast<long double>(in.thickness)*area/4;
        const long double jp=m*in.thickness*in.thickness/12,ja=m*area/12;
        for(unsigned i=0;i<4;++i) { const auto n=rig.element[e].nodes[i];
          mass[n]+=m; inertia[n]+=jp+ja; physical[n]+=jp; added[n]+=ja;
          velocity[n]=interval.velocity_midpoint[i]; omega[n]=interval.omega_midpoint[i]; seen[n]=true;
        }
      }
      for(unsigned n=0;n<rig.n;++n) {
        ASSERT_TRUE(seen[n]); const auto v=velocity[n],w=omega[n];
        const long double vv=static_cast<long double>(v.x)*v.x+static_cast<long double>(v.y)*v.y+static_cast<long double>(v.z)*v.z;
        const long double ww=static_cast<long double>(w.x)*w.x+static_cast<long double>(w.y)*w.y+static_cast<long double>(w.z)*w.z;
        kt+=.5L*mass[n]*vv; kj+=.5L*inertia[n]*ww; kp+=.5L*physical[n]*ww; ka+=.5L*added[n]*ww;
      }
      auto ledger=[](double actual,long double expected) {
        EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),LedgerTolerance*std::max(std::abs(expected),1e-24L));
      };
      ledger(d.kinetic_translation,kt); ledger(d.kinetic_rotation,kj);
      ledger(d.kinetic_physical_isotropic,kp); ledger(d.kinetic_added_isotropic,ka);
      EXPECT_FALSE(d.accepted_force_assembled); EXPECT_EQ(d.internal_kick_work,0.); EXPECT_EQ(d.internal_drift_work,0.);
      EXPECT_EQ(d.base_time,p.base_time); EXPECT_EQ(d.time,p.proposed_time); EXPECT_EQ(d.velocity_time,p.velocity_time);
      EXPECT_EQ(d.kick_dt,p.kick_dt);
      const auto proposed=Bytes(out); ASSERT_TRUE(Commit(rig,token,d));
      q::BatchDiagnostics saved; ASSERT_TRUE(ReadAccepted(rig,out,saved)); EXPECT_EQ(Bytes(out),proposed);
      EXPECT_EQ(saved.phase,q::BatchPhase::Accepted); EXPECT_EQ(saved.attempt,d.attempt);
      EXPECT_EQ(saved.epoch,rig.owner.accepted().epoch);
    }
  }
  RecordProperty("native_cell_intervals",40); RecordProperty("dynamics_qualified","false");
}

TEST_F(QephBatchCuda, CachedPositiveInternalForcesUseNativeSignedSharedScatterWithoutHistoryAdvance) {
  Rig rig(2,true); ASSERT_TRUE(rig.Initialize()); ASSERT_TRUE(rig.Bind());
  fe::NodalTrialToken token; fe::NodalPreparedView p; ASSERT_TRUE(Prepare(rig,Pattern(rig),token,p));
  std::array<q::ForceTrial,4> cache{}; q::BatchDiagnostics d; ASSERT_TRUE(Candidate(rig,p,d,cache));
  ASSERT_TRUE(Commit(rig,token,d)); q::BatchDiagnostics accepted; ASSERT_TRUE(ReadAccepted(rig,cache,accepted));
  const auto cache_bytes=Bytes(cache);
  const auto diagnostic_bytes=Bytes(accepted); Snapshot state; ASSERT_TRUE(Read(rig.owner,state));
  fe::NodalAssemblyView v; ASSERT_EQ(rig.owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  Loads offset=Pattern(rig,-.25); AddLoads<<<1,1,0,v.stream>>>(v,offset,1.);
  ASSERT_EQ(rig.batch.AssembleAccepted(v).status,q::BatchStatus::Success);
  std::array<double,6*N> actual{},truth{};
  double* arrays[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for(unsigned c=0;c<6;++c) {
    ASSERT_EQ(cudaMemcpy(actual.data()+c*N,arrays[c],rig.n*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
    for(unsigned n=0;n<rig.n;++n) truth[c*N+n]=c<3?offset.force[3*n+c]:offset.couple[3*n+c-3];
  }
  double nonzero=0;
  for(unsigned e=0;e<2;++e) {
    const auto& r=cache[e]; double values[24]{},rhs[32]{};
    for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
      values[3*i+a]=q::detail::Component(r.internal_force[i],a);
      values[12+3*i+a]=q::detail::Component(r.internal_couple[i],a); nonzero+=std::abs(values[3*i+a]);
    }
    const double coefficients[]{r.diagnostics.translational_stiffness,r.diagnostics.rotational_stiffness,
                                r.kinematics.nodal_factors[0],r.kinematics.nodal_factors[1]};
    const int nodes[]{1,2,3,4};
    { const std::lock_guard<std::mutex> lock(native::detail::NativeContext()); qeph_q2_scatter(values,coefficients,nodes,rhs); }
    for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
      const auto n=rig.element[e].nodes[i]; truth[a*N+n]+=rhs[3*i+a]; truth[(3+a)*N+n]+=rhs[12+3*i+a];
    }
  }
  EXPECT_GT(nonzero,0.); EXPECT_EQ(actual,truth);
  EXPECT_EQ(rig.batch.AssembleAccepted(v).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(rig.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
  rig.owner.Discard(); rig.batch.DiscardTrial();
  ASSERT_TRUE(ReadAccepted(rig,cache,accepted)); EXPECT_EQ(Bytes(cache),cache_bytes); EXPECT_EQ(Bytes(accepted),diagnostic_bytes);
  Snapshot after; ASSERT_TRUE(Read(rig.owner,after)); SameState(state,after);
}

TEST_F(QephBatchCuda, ImmutableUsageRequiresItsDeclaredForceParticipationBeforeJointPublication) {
  for(auto usage:{q::BatchUsage::PrescribedFields,q::BatchUsage::CoupledForces}) {
    Rig rig(1); ASSERT_TRUE(rig.Initialize(usage)); ASSERT_TRUE(rig.Bind());
    fe::NodalTrialToken token; fe::NodalPreparedView p;
    ASSERT_TRUE(Prepare(rig,Loads{},token,p,usage==q::BatchUsage::PrescribedFields));
    q::BatchDiagnostics d; const auto unchanged=Bytes(d);
    EXPECT_EQ(rig.batch.EvaluateCandidate(p,&d).status,q::BatchStatus::StaleTrial); EXPECT_EQ(Bytes(d),unchanged);
    rig.owner.Discard(); rig.batch.DiscardTrial();
    ASSERT_TRUE(Prepare(rig,Loads{},token,p,usage==q::BatchUsage::CoupledForces));
    std::array<q::ForceTrial,4> result; ASSERT_TRUE(Candidate(rig,p,d,result));
    EXPECT_EQ(d.accepted_force_assembled,usage==q::BatchUsage::CoupledForces);
    EXPECT_EQ(rig.owner.Commit(token).status,fe::NodalStatus::MissingCandidateValidation);
    EXPECT_EQ(rig.owner.accepted().epoch,0u);
    // Owner rejection closes its trial; the material candidate cannot publish.
    EXPECT_NE(q::CommitQephTrial(rig.owner,token,rig.batch,d,Receipt(d)).status,q::BatchStatus::Success);
    ASSERT_TRUE(Prepare(rig,Loads{},token,p,usage==q::BatchUsage::CoupledForces));
    ASSERT_TRUE(Candidate(rig,p,d,result)); ASSERT_TRUE(Commit(rig,token,d)); EXPECT_EQ(rig.owner.accepted().epoch,1u);
  }
}
} // namespace qeph_batch_test
