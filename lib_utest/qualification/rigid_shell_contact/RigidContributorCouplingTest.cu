#include "RigidShellContactFixture.h"
#include "lib_src/elements/qeph/QephForce.h"
#include "lib_src/elements/t3/T3Force.h"
#include <algorithm>

namespace rigid_shell_contact_test {
namespace {
void Near(double actual,double expected) {
  EXPECT_LE(std::abs(actual-expected),2e-12*std::max({1.,std::abs(actual),std::abs(expected)}));
}
tl::math::Vec3 Vector(const std::array<double,3*nt::Capacity>& values,unsigned n) {
  return {values[3*n],values[3*n+1],values[3*n+2]};
}
void Near(tl::math::Vec3 a,tl::math::Vec3 b) { Near(a.x,b.x); Near(a.y,b.y); Near(a.z,b.z); }
}
void CheckForceHistory(Rig& r,const Prepared& p) {
  q::ForceTrial accepted_q; t::ForceTrial accepted_t; q::BatchDiagnostics qd; t::BatchDiagnostics td;
  ASSERT_EQ(r.qeph.CopyAcceptedResults(r.owner.accepted(),&accepted_q,1,&qd).status,q::BatchStatus::Success);
  ASSERT_EQ(r.t3.CopyAcceptedResults(r.owner.accepted(),&accepted_t,1,&td).status,t::BatchStatus::Success);
  const auto endpoint=ReadPrepared(p.view);
  q::PrescribedInterval qi; t::PrescribedInterval ti;
  qi.base_time=ti.base_time=p.view.base_time; qi.dt=ti.dt=H;
  qi.sample_index=ti.sample_index=p.view.kinematics.base_epoch+1;
  for(unsigned i=0;i<4;++i) {
    const unsigned n=r.source.binding.qeph_nodes()[i];
    qi.position_endpoint[i]=Vector(endpoint.x,n); qi.velocity_midpoint[i]=Vector(endpoint.v,n);
    qi.omega_midpoint[i]=Vector(endpoint.omega,n);
  }
  for(unsigned i=0;i<3;++i) {
    const unsigned n=r.source.binding.t3_nodes()[i];
    ti.position[i]=Vector(endpoint.x,n); ti.velocity[i]=Vector(endpoint.v,n); ti.angular_velocity[i]=Vector(endpoint.omega,n);
  }
  // Existing independently qualified host element arithmetic consumes exactly
  // the owner's constrained endpoint; this gate tests contributor gathering.
  q::ForceTrial host_q; t::ForceTrial host_t;
  ASSERT_EQ(q::EvaluateForce(r.source.binding.qeph_reference(),accepted_q.proposed_history,qi,host_q),q::Status::kSuccess);
  ASSERT_EQ(t::EvaluateForce(r.source.binding.t3_reference(),accepted_t.proposed_history,ti,host_t),t::Status::kSuccess);
  for(unsigned i=0;i<4;++i) { Near(p.quad.internal_force[i],host_q.internal_force[i]); Near(p.quad.internal_couple[i],host_q.internal_couple[i]); }
  for(unsigned i=0;i<3;++i) { Near(p.triangle.internal_force[i],host_t.internal_force[i]); Near(p.triangle.internal_couple[i],host_t.internal_couple[i]); }
  EXPECT_EQ(p.quad.proposed_history.stamp().sample_index,qi.sample_index);
  EXPECT_EQ(p.triangle.proposed_history.stamp().sample_index,ti.sample_index);
  EXPECT_EQ(p.quad.proposed_history.stamp().time,p.view.proposed_time);
  EXPECT_EQ(p.triangle.proposed_history.stamp().time,p.view.proposed_time);
}
} // namespace rigid_shell_contact_test

namespace {
using namespace rigid_shell_contact_test;
struct AcceptedShells {
  q::ForceTrial quad; t::ForceTrial triangle; q::BatchDiagnostics qd; t::BatchDiagnostics td;
};
AcceptedShells ReadShells(Rig& r) {
  AcceptedShells out;
  EXPECT_EQ(r.qeph.CopyAcceptedResults(r.owner.accepted(),&out.quad,1,&out.qd).status,q::BatchStatus::Success);
  EXPECT_EQ(r.t3.CopyAcceptedResults(r.owner.accepted(),&out.triangle,1,&out.td).status,t::BatchStatus::Success);
  return out;
}
void SameShells(const AcceptedShells& a,const AcceptedShells& b) {
  EXPECT_EQ(Bytes(a.quad.proposed_history.data()),Bytes(b.quad.proposed_history.data()));
  EXPECT_EQ(Bytes(a.triangle.proposed_history.data()),Bytes(b.triangle.proposed_history.data()));
  for(unsigned n=0;n<4;++n) {
    EXPECT_EQ(Bytes(a.quad.internal_force[n]),Bytes(b.quad.internal_force[n]));
    EXPECT_EQ(Bytes(a.quad.internal_couple[n]),Bytes(b.quad.internal_couple[n]));
  }
  for(unsigned n=0;n<3;++n) {
    EXPECT_EQ(Bytes(a.triangle.internal_force[n]),Bytes(b.triangle.internal_force[n]));
    EXPECT_EQ(Bytes(a.triangle.internal_couple[n]),Bytes(b.triangle.internal_couple[n]));
  }
}
TEST_F(Cuda, NativeWallLoadsTransferThroughGroupAndBothShellHistoriesShareOneCommit) {
  Rig r; ASSERT_TRUE(r.Initialize());
  const auto oa=r.owner.allocations(); const auto qa=r.qeph.allocations();
  const auto ta=r.t3.allocations(); const auto wa=r.contact.allocations();
  double deformation=0;
  for(unsigned step=0;step<4;++step) {
    nt::Snapshot accepted; ASSERT_TRUE(nt::Read(r.owner,accepted)); const auto group=ReadGroup(r.owner);
    const auto materials=ReadShells(r); Prepared p;
    ASSERT_TRUE(Assemble(r,p,step!=0)); const auto load=ReadLoads(p.assembly);
    sc::NodalWallDeviceResults wall;
    ASSERT_EQ(r.contact.CopyResults(p.wall_base,&wall).status,sc::NodalWallDeviceStatus::Ok);
    for(unsigned n=0;n<Nodes;++n) {
      const double expected=1000*r.source.weights.node(n).area.value*std::max(0.,accepted.x[3*n]);
      EXPECT_LE(std::abs(wall.nodes[n].force.value-expected),1e-6);
      EXPECT_EQ(wall.nodes[n].force_world.x,-wall.nodes[n].force.value);
    }
    ASSERT_TRUE(Advance(r,p)); ASSERT_TRUE(Shells(r,p));
    ASSERT_EQ(r.contact.EvaluateCandidate(r.owner,p.token,p.view,&p.wall_candidate).status,sc::NodalWallDeviceStatus::Ok);
    ASSERT_NO_FATAL_FAILURE(CheckForceHistory(r,p));
    fe::NodalRigidGroupSnapshot prepared_group; fe::NodalPreparedView group_view;
    ASSERT_EQ(r.owner.CopyPreparedRigidGroups(p.token,{&prepared_group,1},&group_view).status,fe::NodalStatus::Ok);
    EXPECT_TRUE(fe::trial_identity::SamePrepared(group_view,p.view));
    double net[3]{}; for(unsigned n:{0u,1u,4u}) for(unsigned a=0;a<3;++a) net[a]+=load[a*Nodes+n];
    const double velocity[]{group.state.velocity.x,group.state.velocity.y,group.state.velocity.z};
    const double next[]{prepared_group.state.velocity.x,prepared_group.state.velocity.y,prepared_group.state.velocity.z};
    for(unsigned a=0;a<3;++a) {
      const double expected=velocity[a]+p.view.kick_dt*net[a]/r.source.groups.groups()[0].total_mass_kg;
      EXPECT_LE(std::abs(next[a]-expected),2e-12*std::max(1.,std::abs(expected)));
    }
    const auto endpoint=ReadPrepared(p.view);
    if(step==0) {
      EXPECT_EQ(wall.nodes[0].force.value,0);
      EXPECT_GT(std::abs(endpoint.v[0]),1e-10); // Loaded peers accelerate an uncontacted member.
      EXPECT_LT(prepared_group.state.velocity.x,0);
    }
    for(unsigned n=0;n<4;++n) deformation+=std::abs(p.quad.internal_force[n].x);
    for(unsigned n=0;n<3;++n) deformation+=std::abs(p.triangle.internal_force[n].x);
    nt::Snapshot unchanged; ASSERT_TRUE(nt::Read(r.owner,unchanged)); nt::SameState(accepted,unchanged);
    SameGroup(group,ReadGroup(r.owner)); SameShells(materials,ReadShells(r));
    EXPECT_FALSE(p.shells.qeph.kinetic_available); EXPECT_FALSE(p.shells.t3.kinetic_available);
    ASSERT_TRUE(Commit(r,p)); EXPECT_EQ(r.owner.accepted().epoch,step+1);
    SameGroup(prepared_group,ReadGroup(r.owner));
    EXPECT_EQ(r.owner.rigid_groups().source_instance_id,Source);
    EXPECT_EQ(r.owner.allocations().device_bytes,oa.device_bytes); EXPECT_EQ(r.owner.allocations().device_allocations,oa.device_allocations);
    EXPECT_EQ(r.qeph.allocations().device_bytes,qa.device_bytes); EXPECT_EQ(r.qeph.allocations().device_allocations,qa.device_allocations);
    EXPECT_EQ(r.t3.allocations().device_bytes,ta.device_bytes); EXPECT_EQ(r.t3.allocations().device_allocations,ta.device_allocations);
    EXPECT_EQ(r.contact.allocations().device_bytes,wa.device_bytes); EXPECT_EQ(r.contact.allocations().device_allocations,wa.device_allocations);
  }
  EXPECT_GT(deformation,1e-8);
}
TEST_F(Cuda, LateContactFailureRollsBackGroupNodesAndBothHistoriesWithExactRetry) {
  Rig r,clean; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(clean.Initialize());
  for(Rig* rig:{&r,&clean}) { Prepared p; ASSERT_TRUE(Prepare(*rig,p)); ASSERT_TRUE(Commit(*rig,p)); }
  nt::Snapshot accepted; ASSERT_TRUE(nt::Read(r.owner,accepted)); const auto group=ReadGroup(r.owner);
  const auto materials=ReadShells(r); Prepared failed;
  ASSERT_TRUE(Assemble(r,failed)); ASSERT_TRUE(Advance(r,failed)); ASSERT_TRUE(Shells(r,failed));
  // Inject a late final-node geometry fault into the actual private owner
  // candidate after shell preparation; accepted slabs and source remain intact.
  const double outside=.5;
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(failed.view.kinematics.position_xyz)+3*(Nodes-1),&outside,
      sizeof(outside),cudaMemcpyHostToDevice,failed.view.stream),cudaSuccess);
  sc::NodalWallDiagnostics output; output.attempt=789; const auto before=Bytes(output);
  EXPECT_EQ(r.contact.EvaluateCandidate(r.owner,failed.token,failed.view,&output).status,sc::NodalWallDeviceStatus::GeometryFailure);
  EXPECT_EQ(Bytes(output),before); EXPECT_FALSE(Commit(r,failed,false)); r.Discard();
  nt::Snapshot unchanged; ASSERT_TRUE(nt::Read(r.owner,unchanged)); nt::SameState(accepted,unchanged);
  SameGroup(group,ReadGroup(r.owner)); SameShells(materials,ReadShells(r));
  Prepared retry,reference; ASSERT_TRUE(Prepare(r,retry)); ASSERT_TRUE(Prepare(clean,reference));
  EXPECT_EQ(retry.view.kick_dt,H); EXPECT_EQ(reference.view.kick_dt,H);
  const auto a=ReadPrepared(retry.view),b=ReadPrepared(reference.view);
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.omega,b.omega); EXPECT_EQ(a.q,b.q);
  ASSERT_TRUE(Commit(r,retry)); ASSERT_TRUE(Commit(clean,reference));
  SameGroup(ReadGroup(r.owner),ReadGroup(clean.owner)); SameShells(ReadShells(r),ReadShells(clean));
}
TEST_F(Cuda, GroupedShellsCannotBypassDedicatedAdvanceOrJoinedPublication) {
  Rig r; ASSERT_TRUE(r.Initialize()); nt::Snapshot initial; ASSERT_TRUE(nt::Read(r.owner,initial));
  Prepared p; ASSERT_TRUE(Assemble(r,p)); ASSERT_EQ(r.owner.SealAssembly(p.token).status,fe::NodalStatus::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredHistory(r.owner,p.token,
      {p.assembly.owner_id,0,p.assembly.attempt,H,.2,Qualification}).status,fe::NodalStatus::MissingStepAdmission);
  r.Discard();
  for(unsigned family=0;family<2;++family) {
    Prepared ready; ASSERT_TRUE(Prepare(r,ready));
    const fe::NodalValidationReceipt receipt{ready.view.owner_id,0,ready.view.attempt,Qualification,true};
    if(family==0) EXPECT_EQ(q::CommitQephTrial(r.owner,ready.token,r.qeph,ready.shells.qeph,receipt).status,q::BatchStatus::InvalidInput);
    else EXPECT_EQ(t::CommitT3Trial(r.owner,ready.token,r.t3,ready.shells.t3,receipt).status,t::BatchStatus::InvalidInput);
    r.Discard(); nt::Snapshot unchanged; ASSERT_TRUE(nt::Read(r.owner,unchanged)); nt::SameState(initial,unchanged);
  }
  Prepared good; ASSERT_TRUE(Prepare(r,good)); ASSERT_TRUE(Commit(r,good)); EXPECT_EQ(r.owner.accepted().epoch,1u);
}
} // namespace
