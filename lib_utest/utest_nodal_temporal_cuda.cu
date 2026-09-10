// T1: actual shared-owner timing under known constant WORLD loads only.
// No QEPH history, donor initialization, nonlinear dynamics or step proof.
#include "qualification/nodal/NodalTemporalFixture.h"

namespace {
using namespace tl_test::nodal_temporal;

TEST_F(NodalTemporalCuda, KnownForceStartsAtPhysicalVelocityAndTracksExactMidpoints) {
  Initial in; in.n=3; Loads loads;
  for(unsigned i=0;i<in.n;++i) for(unsigned a=0;a<3;++a) {
    const auto j=3*i+a; in.x[j]=.25*(j+1); in.v[j]=.125*(static_cast<int>(j)-3);
    loads.force[j]=.5*(static_cast<int>(j)-4);
  }
  fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  Snapshot before; ASSERT_TRUE(Read(owner,before)); EXPECT_EQ(before.v,in.v);
  EXPECT_EQ(before.stamp.velocity_phase,Phase::Collocated); EXPECT_EQ(before.stamp.velocity_time,0);
  const auto allocation=owner.allocations(); EXPECT_EQ(allocation.device_allocations,6u);
  for(unsigned step=0;step<32;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
    ASSERT_TRUE(BeginLoad(owner,loads,token,view));
    EXPECT_EQ(view.temporal_scheme,Scheme::StaggeredHalfKickStart);
    EXPECT_EQ(view.position_time,before.stamp.time); EXPECT_EQ(view.velocity_time,before.stamp.velocity_time);
    EXPECT_EQ(view.velocity_phase,before.stamp.velocity_phase);
    ASSERT_TRUE(Prepare(owner,token,view,prepared));
    EXPECT_EQ(prepared.temporal_scheme,view.temporal_scheme);
    EXPECT_EQ(prepared.base_time,before.stamp.time); EXPECT_EQ(prepared.base_velocity_time,before.stamp.velocity_time);
    EXPECT_EQ(prepared.base_velocity_phase,before.stamp.velocity_phase);
    EXPECT_EQ(prepared.velocity_phase,Phase::PreviousMidpoint);
    EXPECT_EQ(prepared.velocity_time,before.stamp.time+.5*in.h);
    EXPECT_EQ(prepared.proposed_time,before.stamp.time+in.h);
    EXPECT_EQ(prepared.kick_dt,step?in.h:.5*in.h);
    Snapshot held; ASSERT_TRUE(Read(owner,held)); SameState(held,before);
    ASSERT_EQ(owner.Commit(token).status,Code::Ok);
    Snapshot next; ASSERT_TRUE(Read(owner,next));
    const double t=(step+1)*in.h,mid=(step+.5)*in.h;
    EXPECT_EQ(next.stamp.time,t); EXPECT_EQ(next.stamp.velocity_time,mid);
    EXPECT_EQ(next.stamp.velocity_phase,Phase::PreviousMidpoint);
    for(unsigned j=0;j<3*in.n;++j) {
      const double acceleration=in.inverse[j/3]*loads.force[j];
      EXPECT_NEAR(next.x[j],in.x[j]+in.v[j]*t+.5*acceleration*t*t,ArithmeticTolerance);
      EXPECT_NEAR(next.v[j],in.v[j]+acceleration*mid,ArithmeticTolerance);
    }
    before=next;
  }
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  // A rounded endpoint minus h/2 need not equal the declared base+h/2.
  // Exercise that distinction, rather than testing only exactly dyadic times.
  in.h=.1; fe::FENodalState decimal; ASSERT_EQ(in.Initialize(decimal).status,Code::Ok);
  bool observed_rounding_difference=false;
  for(unsigned step=0;step<7;++step) {
    const auto base=decimal.accepted(); const double expected=base.time+.5*in.h;
    ASSERT_TRUE(StepConstant(decimal,loads)); const auto accepted=decimal.accepted();
    EXPECT_EQ(accepted.velocity_time,expected);
    observed_rounding_difference|=accepted.velocity_time!=accepted.time-.5*in.h;
  }
  EXPECT_TRUE(observed_rounding_difference);
}

TEST_F(NodalTemporalCuda, ConstantTorqueMatchesIndependentWorldRotationAndKickEnergy) {
  Initial in; in.n=2; in.h=1./64; Loads loads;
  const double r=std::sqrt(.5),axis[3]={2./3,-1./3,2./3};
  for(unsigned i=0;i<in.n;++i) {
    const double sign=i?-1:1; in.q[4*i]=sign*r; in.q[4*i+1]=sign*r;
    for(unsigned a=0;a<3;++a) { in.omega[3*i+a]=.125*axis[a]; loads.couple[3*i+a]=axis[a]; }
  }
  fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  Snapshot before; ASSERT_TRUE(Read(owner,before)); double kick_work=0;
  for(unsigned step=0;step<64;++step) {
    ASSERT_TRUE(StepConstant(owner,loads)); Snapshot after; ASSERT_TRUE(Read(owner,after));
    for(unsigned a=0;a<3;++a)
      kick_work+=after.stamp.reaction_kick_dt*loads.couple[a]*(before.omega[a]+after.omega[a])*.5;
    const double time=(step+1)*in.h,mid=(step+.5)*in.h;
    const double angle=.125*time+.125*time*time,c=std::cos(.5*angle),s=std::sin(.5*angle);
    // Exp_world(angle*axis) times the noncommuting initial x-axis quaternion.
    const double expected[4]={r*(c-axis[0]*s),r*(c+axis[0]*s),
                             r*(axis[1]+axis[2])*s,r*(axis[2]-axis[1])*s};
    for(unsigned i=0;i<in.n;++i) {
      double norm=0;
      for(unsigned a=0;a<4;++a) {
        EXPECT_NEAR(after.q[4*i+a],(i?-1:1)*expected[a],ArithmeticTolerance);
        norm+=after.q[4*i+a]*after.q[4*i+a];
      }
      EXPECT_NEAR(norm,1,1e-15);
      for(unsigned a=0;a<3;++a) EXPECT_NEAR(after.omega[3*i+a],(.125+.25*mid)*axis[a],ArithmeticTolerance);
    }
    before=after;
  }
  double kinetic_change=0;
  for(unsigned a=0;a<3;++a)
    kinetic_change+=.5/in.inverse_inertia[0]*(before.omega[a]*before.omega[a]-in.omega[a]*in.omega[a]);
  EXPECT_NEAR(kick_work,kinetic_change,ArithmeticTolerance);
}

TEST_F(NodalTemporalCuda, FixedMasksKeepBaseReactionsAndSeparateKickFromDriftWork) {
  Initial in; in.n=2; in.fixed[0]=6; in.fixed[1]=7; in.inverse[1]=0;
  in.rotation_fixed[1]=1; in.inverse_inertia[1]=0; in.x[1]=3; in.x[2]=5; in.x[3]=7;
  Loads loads; for(unsigned i=0;i<2;++i) {
    loads.force[3*i]=2; loads.force[3*i+1]=3; loads.force[3*i+2]=4;
    loads.couple[3*i+2]=4;
  }
  fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  Snapshot before; ASSERT_TRUE(Read(owner,before));
  for(unsigned step=0;step<2;++step) {
    ASSERT_TRUE(StepConstant(owner,loads)); Snapshot after; ASSERT_TRUE(Read(owner,after));
    const double kick=step?in.h:.5*in.h;
    EXPECT_EQ(after.stamp.reaction_kick_dt,kick); EXPECT_EQ(after.stamp.reaction_base_epoch,step);
    EXPECT_EQ(after.stamp.reaction_time,before.stamp.time);
    EXPECT_EQ(after.reaction[1],-3); EXPECT_EQ(after.reaction[2],-4);
    EXPECT_EQ(after.reaction[3],-2); EXPECT_EQ(after.reaction[4],-3); EXPECT_EQ(after.reaction[5],-4);
    EXPECT_EQ(after.couple[5],-4); EXPECT_EQ(after.omega[5],0);
    for(unsigned j:{1u,2u,3u,4u,5u}) { EXPECT_EQ(after.x[j],in.x[j]); EXPECT_EQ(after.v[j],0); }
    EXPECT_NEAR(2*(after.v[0]-before.v[0]),2*kick,ArithmeticTolerance);
    EXPECT_NEAR(4*(after.omega[2]-before.omega[2]),4*kick,ArithmeticTolerance);
    const double kinetic=after.v[0]*after.v[0]-before.v[0]*before.v[0]+
                         2*(after.omega[2]*after.omega[2]-before.omega[2]*before.omega[2]);
    const double kick_work=kick*(after.v[0]+before.v[0])+2*kick*(after.omega[2]+before.omega[2]);
    EXPECT_NEAR(kinetic,kick_work,ArithmeticTolerance);
    const double rotation_before=2*std::atan2(before.q[3],before.q[0]);
    const double rotation_after=2*std::atan2(after.q[3],after.q[0]);
    const double drift_work=2*(after.x[0]-before.x[0])+4*(rotation_after-rotation_before);
    if(!step) EXPECT_NEAR(drift_work,4*kinetic,ArithmeticTolerance);
    else EXPECT_NEAR(drift_work-kinetic,3*in.h*in.h,ArithmeticTolerance);
    before=after;
  }
}

TEST_F(NodalTemporalCuda, SchemeAdmissionAndBoundsRejectWithoutConsumingTheHalfKick) {
  Initial in; Loads loads; fe::FENodalState legacy,staggered;
  ASSERT_EQ(in.Initialize(legacy,Scheme::VelocityFirst).status,Code::Ok);
  ASSERT_EQ(in.Initialize(staggered).status,Code::Ok);
  Snapshot unchanged; ASSERT_TRUE(Read(staggered,unchanged));
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_TRUE(BeginLoad(legacy,loads,token,view)); ASSERT_EQ(legacy.SealAssembly(token).status,Code::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredPrescribed(legacy,token,Admission(legacy,view)).status,Code::UnsupportedTemporalScheme);
  for(unsigned variant=0;variant<7;++variant) {
    SCOPED_TRACE(variant); ASSERT_TRUE(BeginLoad(staggered,loads,token,view));
    if(variant==6) AddBound<<<1,1,0,view.stream>>>(view);
    ASSERT_EQ(staggered.SealAssembly(token).status,Code::Ok);
    auto admission=Admission(staggered,view); fe::NodalReport result;
    if(variant<2) {
      fe::NodalStepAdmission old{view.owner_id,view.accepted.base_epoch,view.attempt,in.h,2,
        variant?fe::NodalStepAdmissionKind::RestrictedElasticTrajectory:fe::NodalStepAdmissionKind::PrescribedConstantLoads};
      result=fe::AdvanceNodal(staggered,token,old); EXPECT_EQ(result.status,Code::UnsupportedTemporalScheme);
    } else if(variant==2) {
      result=fe::AdvanceTranslations(staggered,token); EXPECT_EQ(result.status,Code::UnsupportedTemporalScheme);
    } else {
      if(variant==3) admission.owner_id=legacy.accepted().owner_id;
      if(variant==4) ++admission.attempt;
      if(variant==5) admission.maximum_dt=.5*in.h;
      result=fe::AdvanceStaggeredPrescribed(staggered,token,admission);
      EXPECT_EQ(result.status,variant<5?Code::StaleTrial:variant==5?Code::StepTooLarge:Code::MissingStepAdmission);
    }
    staggered.Discard(); Snapshot held; ASSERT_TRUE(Read(staggered,held)); SameState(held,unchanged);
  }
  loads.force[0]=2; ASSERT_TRUE(StepConstant(staggered,loads)); Snapshot after; ASSERT_TRUE(Read(staggered,after));
  EXPECT_EQ(after.v[0],.5*in.h); EXPECT_EQ(after.stamp.reaction_kick_dt,.5*in.h);
  fe::FENodalState invalid;
  EXPECT_EQ(in.Initialize(invalid,static_cast<Scheme>(71)).status,Code::UnsupportedTemporalScheme);
  EXPECT_EQ(invalid.allocations().device_allocations,0u);
  fe::NodalStateConfig c; c.node_count=1; c.temporal_scheme=Scheme::StaggeredHalfKickStart;
  EXPECT_EQ(invalid.Initialize(c,{in.x.data(),in.v.data(),nullptr,1},in.inverse.data(),in.fixed.data()).status,
            Code::UnsupportedTemporalScheme);
  c.fixed_dt=c.minimum_dt=std::numeric_limits<double>::denorm_min();
  EXPECT_EQ(invalid.Initialize(c,{in.x.data(),in.v.data(),in.omega.data(),1,in.q.data()},in.inverse.data(),
      fe::NodalDofConfig{in.fixed.data(),in.rotation_fixed.data(),in.inverse_inertia.data()}).status,Code::InvalidInput);
  EXPECT_EQ(invalid.allocations().device_allocations,0u);
}

TEST_F(NodalTemporalCuda, RealLateOverflowPreservesAcceptedPhaseAndRetryMatchesCleanRun) {
  Initial in; in.n=3; in.h=1; in.x[6]=1e308; in.inverse[2]=1;
  Loads good,bad; good.force[0]=2; bad=good; bad.force[6]=1.7e308;
  fe::FENodalState owner,clean; ASSERT_EQ(in.Initialize(owner).status,Code::Ok); ASSERT_EQ(in.Initialize(clean).status,Code::Ok);
  Snapshot accepted; ASSERT_TRUE(Read(owner,accepted));
  for(unsigned epoch=0;epoch<2;++epoch) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_TRUE(BeginLoad(owner,bad,token,view)); ASSERT_EQ(owner.SealAssembly(token).status,Code::Ok);
    const auto failed=fe::AdvanceStaggeredPrescribed(owner,token,Admission(owner,view));
    EXPECT_EQ(failed.status,Code::InvalidOutput); EXPECT_EQ(failed.node,2u);
    owner.Discard(); Snapshot held; ASSERT_TRUE(Read(owner,held)); SameState(held,accepted);
    ASSERT_TRUE(StepConstant(owner,good)); ASSERT_TRUE(Read(owner,accepted));
    ASSERT_TRUE(StepConstant(clean,good)); Snapshot expected; ASSERT_TRUE(Read(clean,expected));
    expected.stamp.owner_id=accepted.stamp.owner_id; SameState(accepted,expected);
    EXPECT_EQ(accepted.stamp.velocity_time,epoch+.5); EXPECT_EQ(accepted.stamp.reaction_kick_dt,epoch?1:.5);
  }
}

TEST_F(NodalTemporalCuda, PreparedDiscardAndLateRuntimeFailureCannotPublishNewPhaseOrTime) {
  Initial in; Loads loads; loads.force[0]=2;
  fe::FENodalState owner; ASSERT_EQ(in.Initialize(owner).status,Code::Ok); ASSERT_TRUE(StepConstant(owner,loads));
  Snapshot accepted; ASSERT_TRUE(Read(owner,accepted));
  fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
  ASSERT_TRUE(BeginLoad(owner,loads,token,view)); ASSERT_TRUE(Prepare(owner,token,view,prepared));
  Snapshot held; ASSERT_TRUE(Read(owner,held)); SameState(held,accepted);
  owner.Discard(); ASSERT_TRUE(Read(owner,held)); SameState(held,accepted);
  ASSERT_TRUE(BeginLoad(owner,loads,token,view)); ASSERT_TRUE(Prepare(owner,token,view,prepared));
  ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  Noop<<<1,0,0,view.stream>>>(); // Safe invalid launch: no device memory executes.
  const auto error=cudaPeekAtLastError(); ASSERT_TRUE(error==cudaErrorInvalidValue || error==cudaErrorInvalidConfiguration);
  EXPECT_EQ(owner.Commit(token).status,Code::DeviceFailure); SameStamp(owner.accepted(),accepted.stamp);
  EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess);
  EXPECT_EQ(owner.CopyAccepted(held.buffer(),&held.stamp).status,Code::DeviceFailure); SameState(held,accepted);
  fe::FENodalState healthy; ASSERT_EQ(in.Initialize(healthy).status,Code::Ok); ASSERT_TRUE(StepConstant(healthy,loads));
  Snapshot retry; ASSERT_TRUE(Read(healthy,retry)); retry.stamp.owner_id=accepted.stamp.owner_id; SameState(retry,accepted);
}

TEST_F(NodalTemporalCuda, LegacyPacketCapacityUsesTheSameSixAllocationsAndTwoStateSlabs) {
  Initial in; in.n=Capacity; in.h=1./64;
  for(unsigned i=0;i<Capacity;++i) in.v[3*i]=.125*(i+1);
  fe::FENodalState owner,legacy; ASSERT_EQ(in.Initialize(owner).status,Code::Ok);
  ASSERT_EQ(in.Initialize(legacy,Scheme::VelocityFirst).status,Code::Ok);
  const auto allocation=owner.allocations(); EXPECT_EQ(allocation.device_allocations,6u);
  EXPECT_EQ(allocation.device_bytes,legacy.allocations().device_bytes); EXPECT_LE(allocation.device_bytes,32u*1024);
  const double* slab[2]{}; Loads zero;
  for(unsigned step=0;step<64;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view; fe::NodalPreparedView prepared;
    ASSERT_TRUE(BeginLoad(owner,zero,token,view));
    if(step<2) slab[step]=view.accepted.position_xyz; else EXPECT_EQ(view.accepted.position_xyz,slab[step%2]);
    ASSERT_TRUE(Prepare(owner,token,view,prepared)); ASSERT_EQ(owner.Commit(token).status,Code::Ok);
  }
  EXPECT_NE(slab[0],slab[1]); Snapshot result; ASSERT_TRUE(Read(owner,result));
  for(unsigned i=0;i<Capacity;++i) EXPECT_EQ(result.x[3*i],in.v[3*i]);
  EXPECT_EQ(result.stamp.time,1); EXPECT_EQ(result.stamp.velocity_time,1-.5*in.h);
  auto before=result; auto small=result.buffer(); small.capacity_nodes=Capacity-1;
  EXPECT_EQ(owner.CopyAccepted(small,&result.stamp).status,Code::ResourceLimit); SameState(result,before);
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  // Reject before dereferencing the tiny packet: its 64-node storage bound is
  // not the nodal owner's count limit.
  Initial too_large; too_large.n=fe::MaxNodalStateNodes+1; fe::FENodalState rejected;
  EXPECT_EQ(too_large.Initialize(rejected).status,Code::ResourceLimit); EXPECT_EQ(rejected.allocations().device_allocations,0u);
}

TEST_F(NodalTemporalCuda, AcceptedSourceIdentityChecksAllInputsWithoutGrantingAccessOrDrainingCudaErrors) {
  Initial in,other; in.n=other.n=2; other.inverse[0]*=2; other.inverse_inertia[0]*=2; other.x[0]=.5;
  fe::FENodalState owner,foreign,uninitialized,legacy;
  EXPECT_EQ(uninitialized.ValidateAcceptedAssemblySources({}).status,Code::NotInitialized);
  ASSERT_EQ(in.Initialize(owner).status,Code::Ok); ASSERT_EQ(other.Initialize(foreign).status,Code::Ok);
  ASSERT_EQ(in.Initialize(legacy,Scheme::VelocityFirst).status,Code::Ok);
  const auto allocation=owner.allocations(); Snapshot initial; ASSERT_TRUE(Read(owner,initial));
  fe::NodalTrialToken token,other_token,legacy_token; fe::NodalAssemblyView view,other_view,legacy_view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,Code::Ok);
  ASSERT_EQ(foreign.BeginTrial(&other_token,&other_view).status,Code::Ok);
  ASSERT_EQ(legacy.BeginTrial(&legacy_token,&legacy_view).status,Code::Ok);
  EXPECT_EQ(legacy.ValidateAcceptedAssemblySources(legacy_view).status,Code::Ok); legacy.Discard();
  for(unsigned kind=0;kind<20;++kind) {
    SCOPED_TRACE(kind); auto bad=view;
    if(kind==0) bad.mass.inverse_mass=other_view.mass.inverse_mass;
    if(kind==1) bad.mass.fixed=other_view.mass.fixed;
    if(kind==2) bad.inverse_inertia=other_view.inverse_inertia;
    if(kind==3) bad.translation_fixed_bits=other_view.translation_fixed_bits;
    if(kind==4) bad.rotation_fixed=other_view.rotation_fixed;
    if(kind==5) bad.accepted.position_xyz=other_view.accepted.position_xyz;
    if(kind==6) bad.accepted.velocity_xyz=other_view.accepted.velocity_xyz;
    if(kind==7) bad.accepted.angular_velocity_xyz=other_view.accepted.angular_velocity_xyz;
    if(kind==8) bad.accepted.orientation_wxyz=other_view.accepted.orientation_wxyz;
    if(kind==9) bad.stream=other_view.stream;
    if(kind==10) bad.owner_id=other_view.owner_id;
    if(kind==11) ++bad.mass.node_count;
    if(kind==12) ++bad.mass.base_epoch;
    if(kind==13) bad.mass.model=tlfea::contact::TranslationMassModel::kUnspecified;
    if(kind==14) ++bad.accepted.node_count;
    if(kind==15) ++bad.accepted.base_epoch;
    if(kind==16) bad.temporal_scheme=Scheme::VelocityFirst;
    if(kind==17) bad.velocity_phase=Phase::PreviousMidpoint;
    if(kind==18) bad.position_time+=1;
    if(kind==19) bad.velocity_time+=1;
    EXPECT_EQ(owner.ValidateAcceptedAssemblySources(bad).status,Code::StaleTrial);
    EXPECT_EQ(owner.ValidateAcceptedAssemblySources(view).status,Code::Ok);
  }
  auto sources_only=view; sources_only.attempt=0; sources_only.forces={}; sources_only.bounds=nullptr; sources_only.result=nullptr;
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(sources_only).status,Code::Ok);
  // Validation is not attempt authority and cannot make an unadvanced trial
  // ready. Identity comparison alone leaves the complete accepted state intact.
  EXPECT_EQ(owner.Commit(token).status,Code::WrongPhase);
  Snapshot held; ASSERT_TRUE(Read(owner,held)); SameState(initial,held);
  owner.Discard(); foreign.Discard();
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(view).status,Code::Ok);
  fe::NodalAssemblyView current; fe::NodalPreparedView prepared;
  ASSERT_TRUE(BeginLoad(owner,Loads{},token,current)); ASSERT_TRUE(Prepare(owner,token,current,prepared));
  EXPECT_NE(view.attempt,current.attempt);
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(view).status,Code::Ok); // Same accepted epoch, expired view identity only.
  ASSERT_EQ(owner.Commit(token).status,Code::Ok);
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(view).status,Code::StaleTrial);
  ASSERT_TRUE(BeginLoad(owner,Loads{},token,current)); ASSERT_TRUE(Prepare(owner,token,current,prepared));
  const auto stamp=owner.accepted(); ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  Noop<<<1,0,0,current.stream>>>(); const auto pending=cudaPeekAtLastError();
  ASSERT_TRUE(pending==cudaErrorInvalidConfiguration||pending==cudaErrorInvalidValue);
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(current).status,Code::Ok);
  EXPECT_EQ(cudaPeekAtLastError(),pending); // Predicate must leave the error for Commit.
  EXPECT_EQ(owner.Commit(token).status,Code::DeviceFailure); SameStamp(owner.accepted(),stamp);
  EXPECT_EQ(owner.ValidateAcceptedAssemblySources(current).status,Code::DeviceFailure);
  EXPECT_EQ(owner.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_allocations,allocation.device_allocations);
}
} // namespace
