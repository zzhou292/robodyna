// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
namespace controlled_resident_test {
TEST_F(ControlledResidentCuda, AuthenticatedMixedPacketOrAndFailingLaneBarrier) {
  OwnerFixture fixture(false,true,{.001,1000,1},true,true);s::Batch batch;
  ASSERT_EQ(fixture.model.solid24().size(),9u);
  ASSERT_EQ(fixture.model.control_selection()->packets()[0].source.member_count,2u);
  ASSERT_TRUE(Good(batch.InitializeJoined(fixture.Configuration(),fixture.model)));
  const auto& header=Peer::HeaderForPacketProbe(batch);auto* device=Peer::DeviceForPacketProbe(batch);
  const auto& model=fixture.model;const auto n=model.domain()->node_count();
  std::vector<double> x(3*n),v(3*n);
  for(std::size_t j=0;j<n;++j)for(unsigned k=0;k<3;++k)x[3*j+k]=((k==0)?.05:.5)*fe::solid_common::Component(model.domain()->nodes()[j].position,k);
  const auto& distinct=model.solid24()[1];
  v[3*distinct.domain_nodes[0]]=1;v[3*distinct.domain_nodes[1]]=-1;
  v[3*distinct.domain_nodes[5]]=10;v[3*distinct.domain_nodes[7]]=10;
  double* fields=nullptr;ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&fields),6*n*sizeof(double)),cudaSuccess);
  struct Free {double* p;~Free(){cudaFree(p);}} cleanup{fields};
  ASSERT_EQ(cudaMemcpy(fields,x.data(),3*n*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(fields+3*n,v.data(),3*n*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  fe::NodalPreparedView view;view.base_time=0;view.kinematics.base_epoch=0;view.kinematics.position_xyz=fields;view.kinematics.velocity_xyz=fields+3*n;
  d::State<d::Traits24> initial[2],actual[2],unchanged[2];
  ASSERT_EQ(cudaMemcpy(initial,header.solid24.slab[0],sizeof(initial),cudaMemcpyDeviceToHost),cudaSuccess);
  h24::Scratch scratch[2];h24::Result expected[2],isolated;
  for(unsigned p=0;p<2;++p){const auto& parent=model.solid24()[p];h24::Reference ref;
    ASSERT_EQ(h24::PrepareReference(parent.reference,model.materials42()[parent.material_index].value,{.001,1000,1},ref),fe::solid24::ForceStatus::Success);
    auto i=d::Traits24::Phase(0,fixture.Configuration().owner.fixed_dt,0);
    for(unsigned slot=0;slot<8;++slot){const auto j=parent.domain_nodes[slot];d::Traits24::Node(i,slot,{x[3*j],x[3*j+1],x[3*j+2]},{v[3*j],v[3*j+1],v[3*j+2]});}
    ASSERT_NE(initial[p].history.native(),nullptr);
    ASSERT_EQ(h24::PrepareCandidate(ref,*initial[p].history.native(),i,scratch[p]),fe::solid24::ForceStatus::Success);
  }
  ASSERT_GT(scratch[0].activity.flag,0);ASSERT_EQ(scratch[0].activity.triggers_native_batch,0);
  ASSERT_EQ(scratch[1].activity.triggers_native_batch,1);
  for(unsigned p=0;p<2;++p)ASSERT_EQ(h24::Complete(scratch[p],true,expected[p]),fe::solid24::ForceStatus::Success);
  ASSERT_EQ(h24::Complete(scratch[0],false,isolated),fe::solid24::ForceStatus::Success);
  double witness=0;for(unsigned slot=0;slot<8;++slot)witness+=std::abs(expected[0].rhs_force_n[slot].x-isolated.rhs_force_n[slot].x);
  ASSERT_GT(witness,1e-9);ASSERT_GT(expected[0].distortion_work_increment_j-isolated.distortion_work_increment_j,0);
  d::LaunchControlledCandidate(device,0,1,view,header.controlled.blocks);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  int status[2];ASSERT_EQ(cudaMemcpy(status,header.solid24.status,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(status[0],0);ASSERT_EQ(status[1],0);
  ASSERT_EQ(cudaMemcpy(actual,header.solid24.slab[1],sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned p=0;p<2;++p)for(unsigned slot=0;slot<8;++slot)for(unsigned k=0;k<3;++k){
    const auto e=fe::solid_common::Component(expected[p].rhs_force_n[slot],k);
    EXPECT_NEAR(fe::solid_common::Component(actual[p].cache.rhs_force_n[slot],k),e,2e-10*std::max(1.,std::abs(e)));}
  EXPECT_NEAR(actual[0].cache.controlled.distortion_work_j,expected[0].distortion_work_increment_j,2e-10*std::max(1.,std::abs(expected[0].distortion_work_increment_j)));
  // Only the distinct row references slot five. Its failure must still reach
  // the OR/complete barriers; the accepted slab remains byte-for-byte intact.
  x[3*distinct.domain_nodes[5]]=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpy(fields,x.data(),3*n*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  d::LaunchControlledCandidate(device,0,1,view,header.controlled.blocks);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(status,header.solid24.status,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(status[0],0);EXPECT_NE(status[1],0);
  ASSERT_EQ(cudaMemcpy(unchanged,header.solid24.slab[0],sizeof(unchanged),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(std::memcmp(initial,unchanged,sizeof(initial)),0);
}
} // namespace controlled_resident_test
