// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/qeph/QephBatch.h"
#include "lib_src/elements/t3/T3Batch.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
namespace nodal_empty_test {
struct Snapshot {
  std::vector<double> x,v,w,q,force,couple,m,j;double numerical=-1;fe::NodalStamp stamp;
  explicit Snapshot(std::size_t n):x(3*n),v(3*n),w(3*n),q(4*n),force(3*n),couple(3*n),m(n),j(n){}
  void Read(fe::FENodalState& owner) {
    ASSERT_EQ(owner.CopyAccepted({x.data(),v.data(),m.size(),q.data(),w.data(),force.data(),couple.data()},&stamp).status,fe::NodalStatus::Ok);
    fe::NodalStamp raw;
    ASSERT_EQ(owner.CopyAcceptedCin({m.data(),j.data(),nullptr,nullptr,&numerical,m.size(),0},&raw).status,fe::NodalStatus::Ok);
    EXPECT_EQ(raw.epoch,stamp.epoch);
  }
};
struct Rig {
  Fixture f;fe::FENodalState owner;fe::qeph::QephBatch quad;fe::t3::T3Batch triangle;
  fe::qeph::QephBatchConfig qc;fe::t3::T3BatchConfig tc;
  void Initialize() {
    f.Small();auto made=f.Initialize(owner);ASSERT_EQ(made.status,fe::NodalStatus::Ok)<<made.message;
    qc.owner=owner.accepted();qc.configuration_id=901;qc.qualification_id=Fixture::Qualification;
    qc.element_count=f.shells.qeph_count();qc.usage=fe::qeph::BatchUsage::CoupledForces;qc.startup=f.startup;
    tc.owner=owner.accepted();tc.configuration_id=901;tc.qualification_id=Fixture::Qualification;
    tc.element_count=f.shells.t3_count();tc.usage=fe::t3::BatchUsage::CoupledForces;tc.startup=f.startup;
    auto q=quad.InitializeMapped(qc,f.physical,owner,f.Witnesses());
    ASSERT_EQ(q.status,fe::qeph::BatchStatus::Success)<<q.message;
    auto t=triangle.InitializeMapped(tc,f.physical,owner,f.Witnesses());
    ASSERT_EQ(t.status,fe::t3::BatchStatus::Success)<<t.message;
  }
  void Begin(fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    auto q=quad.AssembleMappedAccepted(owner,token,view);ASSERT_EQ(q.status,fe::qeph::BatchStatus::Success)<<q.message;
    auto t=triangle.AssembleMappedAccepted(owner,token,view);ASSERT_EQ(t.status,fe::t3::BatchStatus::Success)<<t.message;
  }
  void Discard(){owner.Discard();quad.DiscardTrial();triangle.DiscardTrial();}
  fe::NodalCinAdmission Admission(const fe::NodalAssemblyView& view) const {
    return {view.owner_id,view.accepted.base_epoch,view.attempt,Fixture::Qualification,
      f.Config().fixed_dt,1.,true,{fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}};
  }
};
TEST(EmptyCinStartupCuda, ActualFixedMasksAuthenticateFreshVelocityAndRejectOldProfiles) {
  Rig rig;ASSERT_NO_FATAL_FAILURE(rig.Initialize());
  ASSERT_EQ(rig.owner.ValidateCinWitnessSource(rig.f.Witnesses()).status,fe::NodalStatus::Ok);
  const auto original=rig.owner.accepted();const auto n=rig.f.mass.size();
  ASSERT_EQ(rig.owner.ValidateInitialConstrainedTranslation(original,rig.f.velocities.data(),n,rig.f.startup.uniform_velocity).status,fe::NodalStatus::Ok);
  for(std::size_t node:{0u,4u,5u}) {
    auto wrong=rig.f.velocities;wrong[3*node+2]+=1;
    EXPECT_EQ(rig.owner.ValidateInitialConstrainedTranslation(original,wrong.data(),n,rig.f.startup.uniform_velocity).status,fe::NodalStatus::InvalidInput);
  }
  auto wrong=rig.f.velocities;wrong[12]=0.;
  EXPECT_EQ(rig.owner.ValidateInitialConstrainedTranslation(original,wrong.data(),n,rig.f.startup.uniform_velocity).status,fe::NodalStatus::InvalidInput);
  auto stale=original;stale.epoch++;
  EXPECT_EQ(rig.owner.ValidateInitialConstrainedTranslation(stale,rig.f.velocities.data(),n,rig.f.startup.uniform_velocity).status,fe::NodalStatus::StaleTrial);
  fe::qeph::QephBatch legacy;auto config=rig.qc;config.startup.kind=fe::ShellBatchStartupKind::ReferenceUniformTranslation;
  EXPECT_NE(legacy.InitializeMapped(config,rig.f.physical,rig.owner,rig.f.Witnesses()).status,fe::qeph::BatchStatus::Success);
  tied::TiedCinAttachmentModel alien;ASSERT_TRUE(tied::PrepareEmptyCinAttachments(rig.f.domain,&alien));
  auto source=rig.f.Witnesses();source.model=&alien;
  EXPECT_EQ(rig.owner.ValidateCinWitnessSource(source).status,fe::NodalStatus::InvalidInput);
  Snapshot snapshot(n);snapshot.Read(rig.owner);EXPECT_EQ(snapshot.m,rig.f.mass);EXPECT_EQ(snapshot.j,rig.f.inertia);
  EXPECT_GT(snapshot.m[0],0);EXPECT_GT(snapshot.j[0],0);EXPECT_EQ(rig.f.inverse_mass[0],0);
}
TEST(EmptyCinStartupCuda, CompleteStiffnessScreenDiscardRetryAndCommitKeepFixedReactions) {
  Rig rig;ASSERT_NO_FATAL_FAILURE(rig.Initialize());const auto n=rig.f.mass.size();
  Snapshot before(n),after(n);before.Read(rig.owner);
  const auto allocations=rig.owner.allocations();
  fe::NodalTrialToken token;fe::NodalAssemblyView view;ASSERT_NO_FATAL_FAILURE(rig.Begin(token,view));
  fe::NodalCinAssemblyView cin;ASSERT_EQ(rig.owner.BorrowCinAssembly(token,&cin).status,fe::NodalStatus::Ok);
  EXPECT_EQ(cin.witness_count,0u);EXPECT_EQ(cin.witness_activity,nullptr);
  std::vector<double> stiffness(n),rotation(n);
  ASSERT_EQ(cudaMemcpyAsync(stiffness.data(),cin.translational_stiffness,n*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(rotation.data(),cin.rotational_stiffness,n*sizeof(double),cudaMemcpyDeviceToHost,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
  for(std::size_t i=0;i<n;++i){EXPECT_GT(stiffness[i],0);EXPECT_GT(rotation[i],0);}
  const double high=1e300;
  ASSERT_EQ(cudaMemcpyAsync(cin.translational_stiffness+4,&high,sizeof(high),cudaMemcpyHostToDevice,cin.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream),cudaSuccess);
  ASSERT_EQ(rig.owner.SealAssembly(token).status,fe::NodalStatus::Ok);
  EXPECT_EQ(fe::AdvanceStaggeredCin(rig.owner,token,rig.Admission(view)).status,fe::NodalStatus::StepTooLarge);
  after.Read(rig.owner);EXPECT_EQ(after.x,before.x);EXPECT_EQ(after.v,before.v);EXPECT_EQ(after.m,before.m);EXPECT_EQ(after.j,before.j);
  rig.Discard();ASSERT_NO_FATAL_FAILURE(rig.Begin(token,view));
  ASSERT_EQ(rig.owner.BorrowCinAssembly(token,&cin).status,fe::NodalStatus::Ok);
  const double load=1000;
  ASSERT_EQ(cudaMemcpyAsync(view.forces.force_x,&load,sizeof(load),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  ASSERT_EQ(rig.owner.SealAssembly(token).status,fe::NodalStatus::Ok);
  const auto advanced=fe::AdvanceStaggeredCin(rig.owner,token,rig.Admission(view));
  ASSERT_EQ(advanced.status,fe::NodalStatus::Ok)<<advanced.message;EXPECT_GT(advanced.stable_dt,rig.f.Config().fixed_dt);
  fe::NodalCinStructuralLimit limit;
  ASSERT_EQ(rig.owner.CopyPreparedCinStructuralLimit(token,&limit).status,fe::NodalStatus::Ok);
  EXPECT_NE(limit.values.kind,fe::NodalCinLimitKind::Unavailable);EXPECT_GE(limit.source_node_id,20u);
  fe::NodalPreparedView raw_prepared;
  ASSERT_EQ(rig.owner.CopyPreparedCin(token,{after.m.data(),after.j.data(),nullptr,nullptr,&after.numerical,n,0},&raw_prepared).status,fe::NodalStatus::Ok);
  EXPECT_EQ(after.m,before.m);EXPECT_EQ(after.j,before.j);EXPECT_EQ(after.numerical,0);
  auto malformed=fe::NodalCinSnapshotBuffer{after.m.data(),after.j.data(),after.m.data(),nullptr,&after.numerical,n,0};
  EXPECT_EQ(rig.owner.CopyPreparedCin(token,malformed,&raw_prepared).status,fe::NodalStatus::InvalidInput);
  fe::NodalPreparedView prepared;
  ASSERT_EQ(rig.owner.CopyPrepared(token,{after.x.data(),after.v.data(),n,after.q.data(),after.w.data(),after.force.data(),after.couple.data()},&prepared).status,fe::NodalStatus::Ok);
  for(std::size_t node=0;node<n;++node)for(unsigned axis=0;axis<3;++axis)
    if(rig.f.fixed[node]&(1u<<axis)){EXPECT_EQ(after.x[3*node+axis],before.x[3*node+axis]);EXPECT_EQ(after.v[3*node+axis],0);}
  EXPECT_EQ(after.force[0],-load);EXPECT_LT(after.x[14],before.x[14]);
  // This owning startup coupon validates nodal motion/reactions, not a material
  // publication claim. Production uses the common physical participant receipt.
  ASSERT_EQ(fe::CompleteNodalValidation(rig.owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,Fixture::Qualification,true}).status,fe::NodalStatus::Ok);
  ASSERT_EQ(rig.owner.Commit(token).status,fe::NodalStatus::Ok);
  after.Read(rig.owner);EXPECT_EQ(after.stamp.epoch,1u);EXPECT_EQ(after.m,before.m);EXPECT_EQ(after.j,before.j);
  EXPECT_EQ(after.numerical,0);EXPECT_EQ(rig.owner.allocations().device_bytes,allocations.device_bytes);
  EXPECT_EQ(rig.owner.allocations().device_allocations,allocations.device_allocations);
}
TEST(EmptyCinStartupCuda, DeclaredEmptyRigidIdentityAndRawFixedMassAreAuthenticated) {
  Fixture f;f.Small();fe::FENodalState declared;
  auto made=f.Initialize(declared);ASSERT_EQ(made.status,fe::NodalStatus::Ok)<<made.message;
  auto copy=f.rigid;EXPECT_EQ(declared.ValidateRigidAssemblyBinding(copy).status,fe::NodalStatus::Ok);
  fe::FENodalState no_cin;
  EXPECT_EQ(no_cin.Initialize(f.Config(),f.Kinematics(),f.inverse_mass.data(),f.Dofs(),f.rigid).status,fe::NodalStatus::InvalidInput);
  auto wrong_scheme=f.Config();wrong_scheme.temporal_scheme=fe::NodalTemporalScheme::VelocityFirst;
  const auto declared_cin=f.Cin();fe::FENodalState velocity_first;
  EXPECT_EQ(velocity_first.Initialize(wrong_scheme,f.Kinematics(),f.inverse_mass.data(),f.Dofs(),f.rigid,&declared_cin).status,fe::NodalStatus::UnsupportedTemporalScheme);

  EXPECT_EQ(declared.accepted().rigid_groups.group_count,0u);
  const auto cin=f.Cin();fe::FENodalState undeclared;
  made=undeclared.Initialize(f.Config(),f.Kinematics(),f.inverse_mass.data(),f.Dofs(),cin);
  ASSERT_EQ(made.status,fe::NodalStatus::Ok)<<made.message;
  EXPECT_NE(undeclared.ValidateRigidAssemblyBinding(copy).status,fe::NodalStatus::Ok);
  EXPECT_EQ(undeclared.allocations().device_bytes,declared.allocations().device_bytes);
  EXPECT_EQ(undeclared.allocations().device_allocations,declared.allocations().device_allocations);
  Fixture other;other.Small();EXPECT_NE(declared.ValidateRigidAssemblyBinding(other.rigid).status,fe::NodalStatus::Ok);
  const std::uint64_t members[]{20,22};
  const fe::rigid::PartTopologyPartInput part{99,members,2};fe::rigid::PartTopologyInput input;
  input.source_instance_id=f.domain.source_instance_id();input.parts=&part;input.part_count=1;
  input.expected_members=members;input.expected_member_count=2;
  fe::rigid::NodalRigidPartTopology topology;ASSERT_TRUE(topology.Initialize(input));
  fe::rigid::NodalRigidPartAssemblyModel parts;ASSERT_TRUE(parts.Initialize(topology,f.ledger,{1000,.001}));
  fe::NodalRigidAssemblyBinding nonempty;ASSERT_TRUE(nonempty.Initialize(parts));
  EXPECT_NE(declared.ValidateRigidAssemblyBinding(nonempty).status,fe::NodalStatus::Ok);
  f.mass[0]*=2;fe::FENodalState wrong_mass;
  EXPECT_EQ(f.Initialize(wrong_mass).status,fe::NodalStatus::InvalidInput);
  f.mass[0]*=.5;f.inertia[0]*=2;fe::FENodalState wrong_inertia;
  EXPECT_EQ(f.Initialize(wrong_inertia).status,fe::NodalStatus::InvalidInput);
  f.inertia[0]*=.5;f.velocities[2]=1;fe::FENodalState moving_fixed;
  EXPECT_EQ(f.Initialize(moving_fixed).status,fe::NodalStatus::InvalidInput);
}

}
