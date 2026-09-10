#include "VehicleWallFixture.h"
namespace vehicle_wall_device_test {
namespace {
__global__ void SetLast(double* field,std::size_t n){field[n]=-std::numeric_limits<double>::max();}
}
TEST_F(CudaTest, LastGlobal359784ScatterFailureLeavesAllForcesAndAcceptedStateUntouchedThenRetriesExactly) {
  auto f=std::make_unique<Fixture>();ASSERT_TRUE(f->Prepare());
  // Only the last node penetrates for this overflow fixture. This keeps the
  // complete global resultant finite while its destination addition overflows.
  for(std::size_t n=0;n+1<f->n;++n)f->x[3*n]=0;
  fe::FENodalState owner;ASSERT_TRUE(f->Owner(owner));sc::NodalWallContactDevice contact;
  auto c=f->Config(owner.accepted());c.law.stiffness_per_area=1e308;
  c.law.parent_force_error=1e305;c.law.parent_energy_error=1e302;
  ASSERT_EQ(contact.Initialize(c,f->wall.view(),f->weights,f->Positions(),f->inverse.data(),f->fixed.data(),f->motion).status,Code::Ok);
  const auto allocations=contact.allocations();std::vector<double> clean,x=f->x,v=f->v;
  Results out(*f);
  for(unsigned pass=0;pass<3;++pass) {
    fe::NodalTrialToken token;fe::NodalAssemblyView a;ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics d;d.owner_id=991;const auto original=nodal_wall_owner_test::Bytes(d);
    if(pass==1) {
      SetLast<<<1,1,0,a.stream>>>(a.forces.force_x,f->n-1);ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
      const auto held=Forces(a);const auto failure=contact.AssembleAccepted(owner,a,&d);
      EXPECT_EQ(failure.status,Code::AssemblyFailure);EXPECT_EQ(failure.node,f->n-1);
      EXPECT_EQ(Forces(a),held);EXPECT_EQ(nodal_wall_owner_test::Bytes(d),original);
      EXPECT_NE(owner.SealAssembly(token).status,fe::NodalStatus::Ok);
    } else {
      ASSERT_EQ(contact.AssembleAccepted(owner,a,&d).status,Code::Ok);
      ASSERT_EQ(contact.CopyResults(d,out.View()).status,Code::Ok);
      EXPECT_GT(out.nodes.back().force.value,1e304);EXPECT_TRUE(std::isfinite(d.resultant.value));
      if(pass==0)clean=Forces(a);else EXPECT_EQ(Forces(a),clean);
    }
    owner.Discard();contact.DiscardTrial();fe::NodalStamp stamp;
    ASSERT_EQ(owner.CopyAccepted({x.data(),v.data(),f->n},&stamp).status,fe::NodalStatus::Ok);
    EXPECT_EQ(x,f->x);EXPECT_EQ(v,f->v);EXPECT_EQ(stamp.epoch,0u);
    EXPECT_EQ(contact.allocations().device_bytes,allocations.device_bytes);EXPECT_EQ(contact.allocations().device_allocations,allocations.device_allocations);
  }
}
} // namespace vehicle_wall_device_test
