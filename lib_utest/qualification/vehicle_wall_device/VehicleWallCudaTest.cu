#include "VehicleWallFixture.h"
#include "../surface_contact/NodalWallCapacityCudaProbe.h"
namespace vehicle_wall_device_test {
TEST_F(CudaTest, Complete349645Parents359785NodesAdvanceWithFullNativeLoadsAndStableAllocations) {
  auto f=std::make_unique<Fixture>();ASSERT_TRUE(f->Prepare());fe::FENodalState owner;
  ASSERT_TRUE(f->Owner(owner));sc::NodalWallContactDevice contact;ASSERT_TRUE(f->Bind(owner,contact));
  const auto allocation=contact.allocations(),nodal=owner.allocations();
  EXPECT_EQ(allocation.device_allocations,1u);EXPECT_EQ(allocation.device_bytes,1078530400u);
  auto x=f->x;Results output(*f);nodal_wall_capacity_probe::CountAllocations(true);
  for(unsigned step=0;step<2;++step) {
    fe::NodalTrialToken token;fe::NodalAssemblyView a;ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics base;ASSERT_EQ(contact.AssembleAccepted(owner,a,&base).status,Code::Ok);
    ASSERT_EQ(contact.CopyResults(base,output.View()).status,Code::Ok);ASSERT_NO_FATAL_FAILURE(CheckLoads(*f,output,x));
    const auto force=Forces(a);
    for(std::size_t n=0;n<f->n;++n){EXPECT_EQ(force[n],output.nodes[n].force_world.x);
      for(unsigned c=1;c<6;++c)EXPECT_EQ(force[c*f->n+n],0);}
    ASSERT_EQ(owner.SealAssembly(token).status,fe::NodalStatus::Ok);
    ASSERT_EQ(fe::AdvanceStaggeredHistory(owner,token,{a.owner_id,a.accepted.base_epoch,a.attempt,H,.1,Qualification}).status,fe::NodalStatus::Ok);
    fe::NodalPreparedView p;ASSERT_EQ(owner.BorrowPrepared(token,&p).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics end;ASSERT_EQ(contact.EvaluateCandidate(owner,token,p,&end).status,Code::Ok);
    ASSERT_EQ(cudaMemcpyAsync(x.data(),p.kinematics.position_xyz,x.size()*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess);
    ASSERT_EQ(contact.CopyResults(end,output.View()).status,Code::Ok);ASSERT_NO_FATAL_FAILURE(CheckLoads(*f,output,x));
    ASSERT_EQ(fe::CompleteNodalValidation(owner,token,{p.owner_id,p.kinematics.base_epoch,p.attempt,Qualification,true}).status,fe::NodalStatus::Ok);
    ASSERT_EQ(owner.Commit(token).status,fe::NodalStatus::Ok);contact.DiscardTrial();
    EXPECT_EQ(contact.allocations().device_bytes,allocation.device_bytes);EXPECT_EQ(contact.allocations().device_allocations,1u);
    EXPECT_EQ(owner.allocations().device_bytes,nodal.device_bytes);EXPECT_EQ(owner.allocations().device_allocations,nodal.device_allocations);
  }
  nodal_wall_capacity_probe::CountAllocations(false);EXPECT_EQ(nodal_wall_capacity_probe::AllocationCalls(),0u);
  EXPECT_EQ(owner.accepted().epoch,2u);EXPECT_EQ(owner.accepted().time,2*H);
  RecordProperty("full_contact_device_bytes",std::to_string(allocation.device_bytes));
  RecordProperty("nodal_owner_device_bytes",std::to_string(nodal.device_bytes));
}
TEST_F(CudaTest, LargeReadbackRejectsLegacyMalformedAndFailedTransfersWithoutOutputPublication) {
  auto f=std::make_unique<Fixture>(2049,73,4097);ASSERT_TRUE(f->Prepare());fe::FENodalState owner;
  ASSERT_TRUE(f->Owner(owner));sc::NodalWallContactDevice contact;ASSERT_TRUE(f->Bind(owner,contact));
  fe::NodalTrialToken token;fe::NodalAssemblyView a;ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  sc::NodalWallDiagnostics d;ASSERT_EQ(contact.AssembleAccepted(owner,a,&d).status,Code::Ok);
  Results out(*f);ASSERT_EQ(contact.CopyResults(d,out.View()).status,Code::Ok);const auto saved=out;
  EXPECT_EQ(contact.CopyResults(d,reinterpret_cast<sc::NodalWallDeviceResults*>(8)).status,Code::ResourceLimit);
  auto bad=out.View();--bad.parent_capacity;bad.parents=reinterpret_cast<sc::NodalWallParentResult*>(8);
  EXPECT_EQ(contact.CopyResults(d,bad).status,Code::InvalidInput);SameResults(out,saved);
  bad=out.View();bad.wall_face=reinterpret_cast<std::uint64_t*>(bad.nodes);
  EXPECT_EQ(contact.CopyResults(d,bad).status,Code::InvalidInput);SameResults(out,saved);
  auto stale=d;++stale.attempt;EXPECT_EQ(contact.CopyResults(stale,out.View()).status,Code::StaleAttempt);SameResults(out,saved);
  nodal_wall_capacity_probe::FailDeviceReadAfter(2);
  EXPECT_EQ(contact.CopyResults(d,out.View()).status,Code::DeviceFailure);SameResults(out,saved);
  EXPECT_EQ(nodal_wall_capacity_probe::DeviceReads(),3u);nodal_wall_capacity_probe::FailDeviceReadAfter(-1);
  EXPECT_EQ(contact.CopyResults(d,out.View()).status,Code::DeviceFailure);SameResults(out,saved);
  owner.Discard();contact.DiscardTrial();
}
} // namespace vehicle_wall_device_test
