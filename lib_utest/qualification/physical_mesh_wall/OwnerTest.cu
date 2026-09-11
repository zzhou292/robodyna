// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/collision/nodal_wall_mapped/Storage.h"
#include <limits>
namespace physical_wall_test {
TEST(PhysicalWallCuda, ExactSourceAndBudgetRejectBeforePublicationThenLoadedRigidCinRetry) {
  p::Rig rig(true);
  ASSERT_TRUE(rig.Initialize());
  Geometry geometry(rig.fixture),wrong(rig.fixture,true);
  ASSERT_EQ(geometry.weights.parent(2).parent_element_id,102u);
  EXPECT_GT(wrong.weights.parent(2).area.value,geometry.weights.parent(2).area.value);
  c::NodalWallMappedContact contact;
  auto config=Config(rig);
  auto source=Source(rig);
  fe::ShellMappedFootprint forecast;
  ASSERT_TRUE(Good(contact.Forecast(config,geometry.weights,source,forecast)));
  const auto before=rig.owner.accepted();
  auto cap=config;
  cap.max_device_bytes=forecast.device_bytes-1;
  EXPECT_EQ(contact.Initialize(cap,geometry.Wall(),geometry.weights,source,rig.owner,geometry.Motion()).status,
      c::NodalWallDeviceStatus::ResourceLimit);
  EXPECT_EQ(contact.allocations().device_bytes,0u);
  auto incomplete=source;
  incomplete.participants.solids=nullptr;
  EXPECT_EQ(contact.Initialize(config,geometry.Wall(),geometry.weights,incomplete,rig.owner,geometry.Motion()).status,
      c::NodalWallDeviceStatus::WrongOwner);
  EXPECT_EQ(contact.Initialize(config,wrong.Wall(),wrong.weights,source,rig.owner,wrong.Motion()).status,
      c::NodalWallDeviceStatus::InvalidInput);
  EXPECT_EQ(contact.allocations().device_bytes,0u);
  ASSERT_TRUE(Good(contact.Initialize(config,geometry.Wall(),geometry.weights,source,rig.owner,geometry.Motion())));
  EXPECT_EQ(contact.allocations().device_bytes,forecast.device_bytes);
  EXPECT_TRUE(fe::trial_identity::SameStamp(before,rig.owner.accepted()));
  ASSERT_NE(rig.fixture.rigid.FindMember(rig.fixture.domain.Find(14)),nullptr);
  p::Snapshot accepted,after;
  ASSERT_TRUE(rig.Read(accepted));
  c::NodalWallMappedDiagnostics rejected;
  for(unsigned interval=0;interval<4;++interval) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics materials,common;
    c::NodalWallMappedDiagnostics base,candidate;
    ASSERT_TRUE(rig.Begin(token,assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(rig.owner.BorrowCinAssembly(token,&cin)));
    const auto node=rig.fixture.domain.Find(14);
    double original=0,added=0;
    ASSERT_EQ(cudaMemcpyAsync(&original,cin.translational_stiffness+node,sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
    auto* alias=reinterpret_cast<c::NodalWallMappedDiagnostics*>(
        const_cast<fe::NodalDomainNode*>(rig.fixture.domain.nodes().data()));
    EXPECT_EQ(contact.AssembleAccepted(rig.owner,token,assembly,alias).status,c::NodalWallDeviceStatus::InvalidInput);
    ASSERT_TRUE(Good(contact.AssembleAccepted(rig.owner,token,assembly,&base)));
    ASSERT_EQ(cudaMemcpyAsync(&added,cin.translational_stiffness+node,sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
    EXPECT_GT(added,original);
    EXPECT_GT(base.contact.resultant.value,0);
    EXPECT_GT(base.current_response_rate_upper,0);
    EXPECT_LT(p::H*std::sqrt(base.current_response_rate_upper),1.6);
    ASSERT_TRUE(rig.Advance(token,assembly,prepared));
    ASSERT_TRUE(rig.Evaluate(token,prepared,materials));
    ASSERT_TRUE(p::Good(rig.publication.PreparePhysical(rig.owner,token,
        {&materials.qeph,&materials.t3,&materials.qbat,&materials.type25,&materials.type13,&materials.solids},&common)));
    ASSERT_TRUE(Good(contact.EvaluateCandidate(rig.owner,token,prepared,common,&candidate)));
    Results result(geometry);
    auto short_view=result.View();
    --short_view.parent_capacity;
    EXPECT_EQ(contact.CopyResults(candidate,short_view).status,c::NodalWallDeviceStatus::InvalidInput);
    EXPECT_FALSE(result.diagnostics.valid);
    ASSERT_TRUE(Good(contact.CopyResults(candidate,result.View())));
    for(const auto& value:result.nodes) EXPECT_FALSE(value.row.valid);
    if(interval==0) {
      rejected=candidate;
      EXPECT_NE(rig.publication.CommitPhysical(rig.owner,token,common,
          {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,p::Qualification,false}).status,
          fe::ShellPublicationStatus::Success);
      contact.DiscardTrial();
      ASSERT_TRUE(rig.Read(after));
      p::Exact(accepted,after);
      EXPECT_EQ(contact.CopyResults(rejected,result.View()).status,c::NodalWallDeviceStatus::StaleAttempt);
    } else {
      ASSERT_TRUE(p::Good(rig.publication.CommitPhysical(rig.owner,token,common,
          {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,p::Qualification,true})));
      EXPECT_EQ(rig.owner.accepted().epoch,interval);
    }
  }
}
TEST(PhysicalWallCuda, LateStiffnessFailureLeavesForceAndAcceptedStateUntouchedThenRetry) {
  p::Rig rig(true);
  ASSERT_TRUE(rig.Initialize());
  Geometry geometry(rig.fixture);
  c::NodalWallMappedContact contact;
  ASSERT_TRUE(Good(contact.Initialize(Config(rig),geometry.Wall(),geometry.weights,Source(rig),rig.owner,geometry.Motion())));
  p::Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly));
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(p::Good(rig.owner.BorrowCinAssembly(token,&cin)));
  const auto last=geometry.weights.node(geometry.weights.node_count()-1).node;
  std::vector<double> force(rig.fixture.domain.node_count()),after_force(force.size());
  ASSERT_EQ(cudaMemcpyAsync(force.data(),assembly.forces.force_x,force.size()*sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),cudaSuccess);
  const double bad=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(cin.translational_stiffness+last,&bad,sizeof(bad),cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
  c::NodalWallMappedDiagnostics output;
  output.accepted_active_parents=777;
  EXPECT_EQ(contact.AssembleAccepted(rig.owner,token,assembly,&output).status,c::NodalWallDeviceStatus::AssemblyFailure);
  EXPECT_EQ(output.accepted_active_parents,777u);
  ASSERT_EQ(cudaMemcpyAsync(after_force.data(),assembly.forces.force_x,force.size()*sizeof(double),cudaMemcpyDeviceToHost,assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
  EXPECT_EQ(force,after_force);
  rig.owner.Discard(); rig.publication.DiscardTrial(); contact.DiscardTrial();
  ASSERT_TRUE(rig.Read(after)); p::Exact(before,after);
  ASSERT_TRUE(rig.Begin(token,assembly));
  ASSERT_TRUE(Good(contact.AssembleAccepted(rig.owner,token,assembly,&output)));
  rig.owner.Discard(); rig.publication.DiscardTrial(); contact.DiscardTrial();
}
} // namespace physical_wall_test
namespace physical_wall_test {
TEST(PhysicalWallCuda, ActualT3RemovalUsesBaseMaskThenPostRemovalZeroAndCaptureRetry) {
  p::Rig rig(false,1e-9);
  ASSERT_TRUE(rig.Initialize());
  Geometry geometry(rig.fixture);
  c::NodalWallMappedContact contact;
  ASSERT_TRUE(Good(contact.Initialize(Config(rig),geometry.Wall(),geometry.weights,Source(rig),rig.owner,geometry.Motion())));
  p::Snapshot initial,after;
  ASSERT_TRUE(rig.Read(initial));
  double retry_removed=0;
  for(unsigned interval=0;interval<4;++interval) {
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics materials,common;
    c::NodalWallMappedDiagnostics base,candidate;
    ASSERT_TRUE(rig.Begin(token,assembly));
    if(interval<2) {
      // Independent half-kick displacement estimate sets a finite source load.
      // The actual owner, weld and T3 recurrence determine the accepted motion.
      const auto node=rig.fixture.domain.Find(14);
      const double force=2*rig.fixture.m[node]*.001/(p::H*p::H);
      ASSERT_EQ(cudaMemcpyAsync(assembly.forces.force_x+node,&force,sizeof(force),cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
      ASSERT_EQ(cudaStreamSynchronize(assembly.stream),cudaSuccess);
    }
    ASSERT_TRUE(Good(contact.AssembleAccepted(rig.owner,token,assembly,&base)));
    ASSERT_TRUE(rig.Advance(token,assembly,prepared));
    ASSERT_TRUE(rig.Evaluate(token,prepared,materials));
    ASSERT_TRUE(p::Good(rig.publication.PreparePhysical(rig.owner,token,
        {&materials.qeph,&materials.t3,&materials.qbat,&materials.type25,&materials.type13,&materials.solids},&common)));
    ASSERT_TRUE(Good(contact.EvaluateCandidate(rig.owner,token,prepared,common,&candidate)));
    Results results(geometry);
    ASSERT_TRUE(Good(contact.CopyResults(candidate,results.View())));
    if(interval<2) {
      EXPECT_EQ(base.accepted_active_parents,4u);
      EXPECT_EQ(candidate.proposed_active_parents,3u);
      EXPECT_GT(candidate.removed_potential.value,0);
      if(!interval) retry_removed=candidate.removed_potential.value;
      else EXPECT_EQ(p::Bits(candidate.removed_potential.value),p::Bits(retry_removed));
    } else {
      EXPECT_EQ(base.accepted_active_parents,3u);
      EXPECT_EQ(candidate.proposed_active_parents,3u);
      EXPECT_EQ(candidate.removed_potential.value,0);
      for(const auto& parent:results.parents) if(parent.parent_element_id==102) {
        EXPECT_EQ(parent.resultant.value,0);
        EXPECT_EQ(parent.potential.value,0);
      }
    }
    ASSERT_TRUE(rig.Read(after));
    if(interval<2) p::Exact(initial,after);
    if(!interval) {
      EXPECT_NE(rig.publication.CommitPhysical(rig.owner,token,common,
          {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,p::Qualification,false}).status,
          fe::ShellPublicationStatus::Success);
      contact.DiscardTrial();
      ASSERT_TRUE(rig.Read(after)); p::Exact(initial,after);
    } else {
      ASSERT_TRUE(p::Good(rig.publication.CommitPhysical(rig.owner,token,common,
          {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,p::Qualification,true})));
    }
  }
}
} // namespace physical_wall_test
