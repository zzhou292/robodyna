// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ValidationCases.h"
#include "lib_src/elements/qbat/activity/Values.h"
#include <cuda_runtime.h>
#include <vector>
namespace qbat_activity_test {
namespace fe=tl::fea;namespace q=fe::qbat;namespace b=q::batch_detail;
struct DeviceFixture {
  qbat_gather_test::Fixture source;
  b::Storage* device=nullptr;
  DeviceFixture(){EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&device),source.layout.bytes),cudaSuccess);}
  ~DeviceFixture(){cudaFree(device);}
  void Restore(unsigned epoch) {
    source.Prepare(epoch,false);
    std::memcpy(device,source.arena.data(),source.layout.bytes);
    *device=source.layout.Rebase(*source.host,device);
  }
  void Check(unsigned slab,double time,std::uint64_t epoch) {
    std::vector<std::uint8_t> expected(qbat_gather_test::Parents,q::activity::InvalidFlag);
    std::uint32_t first=UINT32_MAX;
    for(std::size_t p=0;p<expected.size();++p) {
      const bool valid=b::ValidResult(device->slab[slab].element[p],device->model.element[p].material,time,epoch);
      if(valid) expected[p]=device->slab[slab].element[p].history.element_active?1:0;
      else if(first==UINT32_MAX) first=static_cast<std::uint32_t>(p);
    }
    ASSERT_EQ(cudaMemset(device->activity.first_invalid,0xff,sizeof(std::uint32_t)),cudaSuccess);
    ASSERT_EQ(cudaMemset(device->activity.active,73,expected.size()),cudaSuccess);
    b::LaunchParentActivity(device,expected.size(),slab,time,epoch,nullptr);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    EXPECT_EQ(*device->activity.first_invalid,first);
    for(std::size_t p=0;p<expected.size();++p) EXPECT_EQ(device->activity.active[p],expected[p])<<p;
  }
};
TEST(QbatCompactActivityCuda,StartupCarriedRemovedAndFreshRetryMatchCompleteCpuPredicate) {
  DeviceFixture fixture;
  for(unsigned epoch:{0u,1u,3u}) for(unsigned slab:{0u,1u}) {
    for(unsigned mask=0;mask<16;++mask) {
      fixture.Restore(epoch);
      for(unsigned p=0;p<4;++p) if(mask&(1u<<p)) qbat_gather_test::Remove(fixture.device->slab[slab].element[p]);
      fixture.Check(slab,(epoch+slab)*qbat_gather_test::Dt,epoch+slab);
    }
  }
  fixture.Restore(2);fixture.Check(1,3*qbat_gather_test::Dt,3);
}
TEST(QbatCompactActivityCuda,EveryResultFieldFamilyAndExactTimeEpochRemainValidated) {
  DeviceFixture fixture;
  for(unsigned epoch:{0u,2u}) for(unsigned slab:{0u,1u}) for(unsigned fault=0;fault<FaultCount;++fault) {
    SCOPED_TRACE(::testing::Message()<<epoch<<':'<<slab<<':'<<fault);
    fixture.Restore(epoch);
    Corrupt(fixture.device->slab[slab].element[3],fault);
    fixture.Check(slab,(epoch+slab)*qbat_gather_test::Dt,epoch+slab);
    EXPECT_EQ(*fixture.device->activity.first_invalid,3u);
  }
}
TEST(QbatCompactActivityCuda,EarliestFailureIsDeterministicAndAllRowsAreFreshlyOverwritten) {
  DeviceFixture fixture;
  for(unsigned repeat=0;repeat<32;++repeat) {
    fixture.Restore(2);
    Corrupt(fixture.device->slab[1].element[3],repeat%FaultCount);
    Corrupt(fixture.device->slab[1].element[1],(repeat+3)%FaultCount);
    fixture.Check(1,3*qbat_gather_test::Dt,3);
    EXPECT_EQ(*fixture.device->activity.first_invalid,1u);
    fixture.Restore(2);fixture.Check(1,3*qbat_gather_test::Dt,3);
    EXPECT_EQ(*fixture.device->activity.first_invalid,UINT32_MAX);
  }
}
TEST(QbatCompactActivityCuda, MaterialRateAndTabulatedDomainAreFreshPredicateInputs) {
  DeviceFixture fixture;
  fixture.Restore(0);
  auto& rate=fixture.device->model.element[3].material;
  rate.rate.enabled=false;
  auto& history=fixture.device->slab[0].element[3];
  history.history.point[3].material.filtered_rate_per_s=1;
  fixture.Check(0,0,0);
  EXPECT_EQ(*fixture.device->activity.first_invalid,3u);
  double* curve=nullptr;
  ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&curve),4*sizeof(double)),cudaSuccess);
  curve[0]=0;curve[1]=1;curve[2]=10e6;curve[3]=11e6;
  fixture.Restore(0);
  auto& tabulated=fixture.device->model.element[3].material;
  tabulated.hardening=tl::material::ShellPlasticityHardeningKind::Tabulated;
  tabulated.continuation=tl::material::ShellPlasticityCurveContinuation::StrictDomain;
  tabulated.curve={curve,curve+2,2};
  fixture.device->slab[0].element[3].history.point[3].material.plastic_strain=2;
  fixture.Check(0,0,0);
  EXPECT_EQ(*fixture.device->activity.first_invalid,3u);
  ASSERT_EQ(cudaFree(curve),cudaSuccess);
}
TEST(QbatCompactActivityCuda, CompletePacketSpansBlockBoundariesAndKeepsTheLastSourceIndex) {
  qbat_gather_test::Fixture seed;
  seed.Prepare(0,false);
  for(std::size_t parents:{1u,127u,128u,129u,263u,4250u}) {
    b::Layout layout;
    ASSERT_TRUE(layout.Initialize(parents,4,0,64u<<20));
    tl::util::HostArena host;
    ASSERT_TRUE(host.Initialize(layout.bytes));
    auto* state=layout.Construct(host);ASSERT_NE(state,nullptr);
    state->model.config.element_count=parents;
    for(std::size_t parent=0;parent<parents;++parent) {
      state->model.element[parent]=seed.host->model.element[0];
      state->model.element[parent].source_parent_id=200+parent;
      state->slab[0].element[parent]=seed.host->slab[0].element[0];
    }
    Corrupt(state->slab[0].element[parents-1],FaultCount-1);
    b::Storage* device=nullptr;
    ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&device),layout.bytes),cudaSuccess);
    std::memcpy(device,host.data(),layout.bytes);*device=layout.Rebase(*state,device);
    ASSERT_EQ(cudaMemset(device->activity.first_invalid,0xff,sizeof(std::uint32_t)),cudaSuccess);
    ASSERT_EQ(cudaMemset(device->activity.active,73,parents),cudaSuccess);
    b::LaunchParentActivity(device,parents,0,0,0,nullptr);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    EXPECT_EQ(*device->activity.first_invalid,parents-1);
    for(std::size_t parent=0;parent<parents;++parent)
      EXPECT_EQ(device->activity.active[parent],parent+1==parents?q::activity::InvalidFlag:1)<<parent;
    ASSERT_EQ(cudaFree(device),cudaSuccess);
  }
}
} // namespace qbat_activity_test
