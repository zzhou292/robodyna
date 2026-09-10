#include "MixedShellLedger.h"

namespace mixed_shell_test {
namespace {
void SameKinetic(const fe::ShellBatchKinetic& a,const fe::ShellBatchKinetic& b) {
  EXPECT_EQ(a.translation,b.translation); EXPECT_EQ(a.rotation,b.rotation);
  EXPECT_EQ(a.physical_isotropic,b.physical_isotropic); EXPECT_EQ(a.added_isotropic,b.added_isotropic);
}
std::array<fe::NodalAllocationInfo,4> Allocations(const Rig& r) {
  return {r.owner.allocations(),r.qeph.allocations(),r.t3.allocations(),r.publication.allocations()};
}
void SameAllocations(const Rig& r,const std::array<fe::NodalAllocationInfo,4>& expected) {
  const auto actual=Allocations(r);
  for(unsigned i=0;i<actual.size();++i) {
    EXPECT_EQ(actual[i].device_bytes,expected[i].device_bytes);
    EXPECT_EQ(actual[i].device_allocations,expected[i].device_allocations);
  }
}
}
TEST_F(MixedShellCuda, NativeSharedEdgeBindingUsesOneOwnerMassAndSeparateTypedHistories) {
  Rig r; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  ASSERT_NO_FATAL_FAILURE(shell_binding_test::CheckTruth(r.input,r.binding));
  ASSERT_NO_FATAL_FAILURE(shell_binding_test::CheckNativeReduction(r.input,r.binding));
  NativePair native; ASSERT_TRUE(native.Initialize(r));
  Snapshot state; ASSERT_TRUE(Read(r.owner,state));
  EXPECT_EQ(state.stamp.node_count,Nodes); EXPECT_EQ(state.stamp.epoch,0u);
  EXPECT_EQ(state.stamp.time,0); EXPECT_EQ(state.stamp.velocity_time,0);
  EXPECT_EQ(state.stamp.velocity_phase,fe::NodalVelocityPhase::Collocated);
  EXPECT_EQ(state.stamp.temporal_scheme,fe::NodalTemporalScheme::StaggeredHalfKickStart);
  for(unsigned n=0;n<Nodes;++n) {
    for(unsigned a=0;a<3;++a) {
      EXPECT_EQ(state.x[3*n+a],r.initial.x[3*n+a]); EXPECT_EQ(state.v[3*n+a],0);
      EXPECT_EQ(state.omega[3*n+a],0); EXPECT_EQ(state.q[4*n+1+a],0);
    }
    EXPECT_EQ(state.q[4*n],1);
    EXPECT_NEAR(r.initial.inverse[n]*r.binding.nodes()[n].native.mass,1,1e-12);
    EXPECT_NEAR(r.initial.inverse_inertia[n]*r.binding.nodes()[n].native.isotropic_inertia,1,1e-12);
  }
  EXPECT_EQ(r.binding.nodes()[4].source_id,shell_binding_test::WideId);
  EXPECT_GT(r.binding.nodes()[1].native.mass,r.binding.qeph_reference().nodal_mass[1]);
  EXPECT_GT(r.binding.nodes()[1].native.mass,r.binding.t3_reference().nodal_mass[0]);
  Staged accepted; ASSERT_TRUE(Accepted(r,accepted));
  auto initial=[](const auto& d) {
    EXPECT_TRUE(d.valid); EXPECT_FALSE(d.has_completed_interval); EXPECT_FALSE(d.accepted_force_assembled);
    EXPECT_FALSE(d.kinetic_available); EXPECT_EQ(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
    EXPECT_EQ(d.kinetic_physical_isotropic,0); EXPECT_EQ(d.kinetic_added_isotropic,0);
    EXPECT_EQ(d.epoch,0u); EXPECT_EQ(d.minimum_native_dt,0);
  };
  initial(accepted.diagnostics.qeph); initial(accepted.diagnostics.t3);
  EXPECT_TRUE(accepted.qeph.proposed_history.matches_reference(r.binding.qeph_reference()));
  EXPECT_TRUE(accepted.t3.proposed_history.matches_reference(r.binding.t3_reference()));
  EXPECT_EQ(accepted.qeph.proposed_history.stamp().sample_index,0u);
  EXPECT_EQ(accepted.t3.proposed_history.stamp().sample_index,0u);
  for(unsigned i=0;i<2;++i) {
    EXPECT_EQ(accepted.qeph.proposed_history.data().internal_work[i],0);
    EXPECT_EQ(accepted.t3.proposed_history.data().internal_work[i],0);
  }
  EXPECT_EQ(accepted.qeph.proposed_history.data().hourglass_viscous_work,0);
  for(const auto& f:accepted.qeph.internal_force) EXPECT_EQ(qeph_startup_test::Length(f),0);
  for(const auto& f:accepted.qeph.internal_couple) EXPECT_EQ(qeph_startup_test::Length(f),0);
  for(const auto& f:accepted.t3.internal_force) EXPECT_EQ(qeph_startup_test::Length(f),0);
  for(const auto& f:accepted.t3.internal_couple) EXPECT_EQ(qeph_startup_test::Length(f),0);
  const auto allocations=Allocations(r);
  EXPECT_EQ(allocations[0].device_allocations,6u);
  for(unsigned i=1;i<allocations.size();++i) EXPECT_EQ(allocations[i].device_allocations,1u);
  EXPECT_LE(allocations[0].device_bytes,fe::MaxTranslationDeviceBytes);
  EXPECT_LE(allocations[1].device_bytes,q::MaxBatchDeviceBytes);
  EXPECT_LE(allocations[2].device_bytes,t::MaxBatchDeviceBytes);
  EXPECT_LE(allocations[3].device_bytes,8192u); // Owning collection kinetic scratch's fixed cap.
  std::size_t total=0;
  for(unsigned i=0;i<allocations.size();++i) {
    EXPECT_GT(allocations[i].device_bytes,0u); total+=allocations[i].device_bytes;
    RecordProperty("owned_device_bytes_"+std::to_string(i),std::to_string(allocations[i].device_bytes));
  }
  RecordProperty("total_owned_device_bytes",std::to_string(total));
  RecordProperty("owner_count",1); RecordProperty("physical_nodes",Nodes);
}

TEST_F(MixedShellCuda, PrescribedLoadHoldReversePublishesBothNativeHistoriesAndOneKineticLedger) {
  Rig r; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  NativePair native; ASSERT_TRUE(native.Initialize(r));
  const auto allocations=Allocations(r);
  Staged accepted; ASSERT_TRUE(Accepted(r,accepted));
  double maximum_qwork[2]{},maximum_twork[2]{},maximum_qforce=0,maximum_tforce=0;
  for(unsigned interval=0;interval<4;++interval) {
    SCOPED_TRACE(interval);
    Snapshot base; ASSERT_TRUE(Read(r.owner,base));
    Prepared p; ASSERT_TRUE(Prepare(r,Schedule(r,interval),p)); CheckTargets(r,interval,p.endpoint);
    Staged next; ASSERT_TRUE(Evaluate(r,p,next,interval%2)); Identity(r,p,next);
    NativeTrials expected; ASSERT_TRUE(native.Check(r,p,next,expected));
    CheckLedgers(r,base,accepted,p,next);
    for(unsigned i=0;i<2;++i) {
      maximum_qwork[i]=std::max(maximum_qwork[i],std::abs(next.diagnostics.qeph.internal_work[i]));
      maximum_twork[i]=std::max(maximum_twork[i],std::abs(next.diagnostics.t3.internal_work[i]));
    }
    for(auto f:next.qeph.internal_force) maximum_qforce=std::max(maximum_qforce,qeph_startup_test::Length(f));
    for(auto f:next.t3.internal_force) maximum_tforce=std::max(maximum_tforce,qeph_startup_test::Length(f));
    if(interval==3) {
      // Frozen inherited TB2 signal floors; prevent a zero mixed history from
      // passing only because its identity/publication path is well formed.
      EXPECT_GT(maximum_qwork[0],1e-12); EXPECT_GT(maximum_qwork[1],1e-14);
      EXPECT_GT(maximum_twork[0],1e-12); EXPECT_GT(maximum_twork[1],1e-14);
      EXPECT_GT(maximum_qforce,1e-5); EXPECT_GT(maximum_tforce,1e-5);
    }
    ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Publish(r,p,next)); native.Accept(expected);
    Staged copied; ASSERT_TRUE(Accepted(r,copied)); ExactResults(copied,next);
    EXPECT_EQ(copied.diagnostics.qeph.phase,q::BatchPhase::Accepted);
    EXPECT_EQ(copied.diagnostics.t3.phase,t::BatchPhase::Accepted);
    fe::ShellBatchDiagnostics global;
    ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&global).status,fe::ShellPublicationStatus::Success);
    EXPECT_TRUE(global.valid); SameKinetic(global.kinetic,next.diagnostics.kinetic);
    SameKinetic(global.base_kinetic,next.diagnostics.base_kinetic);
    EXPECT_EQ(global.qeph.epoch,interval+1); EXPECT_EQ(global.t3.epoch,interval+1);
    EXPECT_EQ(global.qeph.attempt,p.view.attempt); EXPECT_EQ(global.t3.attempt,p.view.attempt);
    Snapshot state; ASSERT_TRUE(Read(r.owner,state));
    EXPECT_EQ(state.x,p.endpoint.x); EXPECT_EQ(state.v,p.endpoint.v);
    EXPECT_EQ(state.omega,p.endpoint.omega); EXPECT_EQ(state.q,p.endpoint.q);
    EXPECT_EQ(state.stamp.epoch,interval+1); EXPECT_EQ(state.stamp.time,(interval+1)*H);
    EXPECT_EQ(state.stamp.velocity_time,(interval+.5)*H);
    EXPECT_EQ(state.stamp.reaction_kick_dt,interval?H:.5*H);
    SameAllocations(r,allocations); accepted=next;
  }
  Property("maximum_qeph_membrane_work_J",maximum_qwork[0]); Property("maximum_qeph_bending_work_J",maximum_qwork[1]);
  Property("maximum_t3_membrane_work_J",maximum_twork[0]); Property("maximum_t3_bending_work_J",maximum_twork[1]);
  Property("maximum_qeph_force_N",maximum_qforce); Property("maximum_t3_force_N",maximum_tforce);
  RecordProperty("native_qeph_intervals",4); RecordProperty("native_t3_intervals",4);
  RecordProperty("aggregate_energy_scale_J","0.000001"); RecordProperty("mixed_coupled_response_qualified","false");
}

TEST_F(MixedShellCuda, BothAcceptedCachesMatchNativeSignedScatterInDisposableSharedAssembly) {
  Rig r; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  NativePair native; ASSERT_TRUE(native.Initialize(r));
  Snapshot initial; ASSERT_TRUE(Read(r.owner,initial)); Staged accepted; ASSERT_TRUE(Accepted(r,accepted));
  Prepared p; ASSERT_TRUE(Prepare(r,Schedule(r,0),p)); CheckTargets(r,0,p.endpoint);
  Staged next; ASSERT_TRUE(Evaluate(r,p,next)); NativeTrials expected; ASSERT_TRUE(native.Check(r,p,next,expected));
  CheckLedgers(r,initial,accepted,p,next);
  ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Publish(r,p,next)); native.Accept(expected);
  ASSERT_TRUE(Accepted(r,accepted));
  const auto saved_q=Bytes(accepted.qeph);
  const auto saved_t=Bytes(accepted.t3);
  const auto saved_qd=Bytes(accepted.diagnostics.qeph);
  const auto saved_td=Bytes(accepted.diagnostics.t3);
  Snapshot before; ASSERT_TRUE(Read(r.owner,before));
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  Loads seed;
  for(unsigned n=0;n<Nodes;++n) for(unsigned a=0;a<3;++a) {
    seed.force[3*n+a]=.125*(1+3*n+a); seed.couple[3*n+a]=-.0625*(1+3*n+a);
  }
  temporal::AddLoads<<<1,1,0,view.stream>>>(view,seed,1.);
  ASSERT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
  ASSERT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::Success);
  CheckNativeScatter(r,accepted,seed,view);
  const auto once=Assembly(view);
  EXPECT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(Assembly(view),once);
  EXPECT_EQ(r.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
  r.Discard();
  Staged after; ASSERT_TRUE(Accepted(r,after));
  EXPECT_EQ(Bytes(after.qeph),saved_q); EXPECT_EQ(Bytes(after.t3),saved_t);
  EXPECT_EQ(Bytes(after.diagnostics.qeph),saved_qd); EXPECT_EQ(Bytes(after.diagnostics.t3),saved_td);
  Snapshot state; ASSERT_TRUE(Read(r.owner,state)); SameState(before,state);
  RecordProperty("native_qeph_intervals",1); RecordProperty("native_t3_intervals",1);
  RecordProperty("mixed_coupled_response_qualified","false");
}
} // namespace mixed_shell_test
