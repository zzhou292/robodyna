// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
#include "lib_src/elements/solids/resident/AssemblySerial.cuh"
namespace controlled_resident_test {
__global__ void SerialAssembly(d::Storage* storage,fe::NodalAssemblyView a,fe::NodalCinAssemblyView cin){
  if(!threadIdx.x&&!blockIdx.x)d::assembly_serial::Assemble(storage,0,a,cin);
}
TEST_F(ControlledResidentCuda, CollapsedH24GatherSerialAndForcedFallbackPreserveSeededBits) {
  Rig rig({.001,1000,1},true);ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;fe::NodalAssemblyView a;fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(Good(rig.owner.BeginTrial(&token,&a)));ASSERT_TRUE(Good(rig.owner.BorrowCinAssembly(token,&cin)));
  auto* device=Peer::DeviceForPacketProbe(rig.batch);const auto& header=Peer::HeaderForPacketProbe(rig.batch);
  const auto n=rig.config.owner.node_count;std::vector<double> seeds[4],expected[4],actual(n),sentinel(n);
  for(unsigned k=0;k<4;++k){seeds[k].resize(n);expected[k].resize(n);for(std::size_t j=0;j<n;++j)seeds[k][j]=k==3?.125:((j+k)%2?-.123:.234);}
  for(std::size_t j=0;j<n;++j)sentinel[j]=j%2?-0.0:17.;
  double* fields[]{a.forces.force_x,a.forces.force_y,a.forces.force_z,cin.translational_stiffness};
  double* untouched[]{a.forces.couple_x,a.forces.couple_y,a.forces.couple_z,cin.rotational_stiffness};
  auto seed=[&](){for(unsigned k=0;k<4;++k)if(cudaMemcpy(fields[k],seeds[k].data(),n*sizeof(double),cudaMemcpyHostToDevice)!=cudaSuccess)return false;
    for(auto* p:untouched)if(cudaMemcpy(p,sentinel.data(),n*sizeof(double),cudaMemcpyHostToDevice)!=cudaSuccess)return false;return true;};
  ASSERT_TRUE(seed());ASSERT_TRUE(Good(rig.batch.AssembleAccepted(rig.owner,token,a)));
  for(unsigned k=0;k<4;++k)ASSERT_EQ(cudaMemcpy(expected[k].data(),fields[k],n*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
  for(unsigned mode=0;mode<2;++mode){
    SCOPED_TRACE(mode);ASSERT_TRUE(seed());
    if(!mode)SerialAssembly<<<1,1,0,a.stream>>>(device,a,cin);
    else {
      const std::size_t zero=0;ASSERT_EQ(cudaMemcpy(&device->assembly.arena_bytes,&zero,sizeof(zero),cudaMemcpyHostToDevice),cudaSuccess);
      ASSERT_EQ(d::LaunchAssembly(device,0,a,cin),cudaSuccess);
    }
    ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
    d::Control status;ASSERT_EQ(cudaMemcpy(&status,&device->control,sizeof(status),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(status.status,s::BatchStatus::Success);
    if(mode){unsigned fallback=0;ASSERT_EQ(cudaMemcpy(&fallback,&device->assembly.fallback,sizeof(fallback),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(fallback,1u);
      ASSERT_EQ(cudaMemcpy(&device->assembly.arena_bytes,&header.assembly.arena_bytes,sizeof(std::size_t),cudaMemcpyHostToDevice),cudaSuccess);}
    for(unsigned k=0;k<4;++k){ASSERT_EQ(cudaMemcpy(actual.data(),fields[k],n*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(std::memcmp(actual.data(),expected[k].data(),n*sizeof(double)),0)<<k;}
    for(auto* p:untouched){ASSERT_EQ(cudaMemcpy(actual.data(),p,n*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);EXPECT_EQ(std::memcmp(actual.data(),sentinel.data(),n*sizeof(double)),0);}
  }
  rig.owner.Discard();rig.batch.DiscardTrial();
}
} // namespace controlled_resident_test
