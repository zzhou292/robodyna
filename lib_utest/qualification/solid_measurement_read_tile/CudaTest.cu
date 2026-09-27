// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Support.h"
#include "CurrentFinalize.cuh"
#include <cuda_runtime.h>
#include <new>
namespace solid_read_tile_test {
__global__ void Frozen(b::Storage* state,fe::NodalPreparedView view,s::BatchDiagnostics seed,bool initial) {
  b::read_tile_reference::Finalize(state,0,1,view,seed,initial,true);
}
struct Rig {
  b::Storage* state=nullptr;
  std::vector<void*> allocated;
  template<class T>T* Allocate(std::size_t count) {
    if(!count)return nullptr;
    T* value=nullptr;EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&value),count*sizeof(T)),cudaSuccess);
    if(!value)return nullptr;allocated.push_back(value);
    for(std::size_t i=0;i<count;++i)new (value+i) T{};
    return value;
  }
  template<class Traits>void Family(std::size_t count) {
    auto& f=b::FamilyStorage<Traits>(*state);f.count=count;f.status=Allocate<int>(count);
    f.result_valid=Allocate<std::uint8_t>(count);f.measurement=Allocate<b::MeasurementOperands<Traits::nodes>>(count);
    for(std::size_t p=0;p<count;++p) {
      f.result_valid[p]=1;auto& v=f.measurement[p];v.work=.25;v.hourglass_work=-.125;v.plastic_work=.125;v.native_dt=.01;
      for(unsigned n=0;n<Traits::nodes;++n) {v.kick[n]=.5*(1+n);v.drift[n]=-.25*(1+n);}
    }
  }
  explicit Rig(std::size_t count) {
    state=Allocate<b::Storage>(1);if(!state)return;
    state->source_instance_id=101;state->config.owner.owner_id=103;state->config.configuration_id=107;state->config.qualification_id=109;
    Family<b::Traits18>(count);Family<b::Traits24>(count);Family<b::Traits6z>(count);
    Family<b::Traits18Law44>(count);Family<b::Traits18Law90>(count);
  }
  ~Rig(){for(auto p:allocated)cudaFree(p);}
  b::Control Compare(bool initial=false,s::BatchDiagnostics seed=Seed(true,-0.)) {
    state->control=Poison();Frozen<<<1,1>>>(state,{},seed,initial);Drain();const auto expected=state->control;
    state->control=Poison();b::read_tile_current::Finalize<<<1,tile::Threads>>>(state,0,1,{},seed,initial,true);Drain();
    Same(state->control,expected);return state->control;
  }
  static void Drain(){ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);}
};
TEST(SolidMeasurementReadTileCuda, EmptyAndPartialTilesAcrossAllFiveFamilies) {
  for(std::size_t count:{0u,1u,31u,32u,33u,63u,64u,65u,127u,128u,129u}) {
    SCOPED_TRACE(count);Rig rig(count);ASSERT_FALSE(HasFailure());
    for(bool initial:{false,true})for(bool valid:{false,true}) {
      EXPECT_EQ(rig.Compare(initial,Seed(valid,-0.)).status,s::BatchStatus::Success);ASSERT_FALSE(HasFailure());
    }
  }
}
template<class Traits>void BadRows(unsigned index) {
  Rig rig(129);ASSERT_FALSE(::testing::Test::HasFailure());auto& family=b::FamilyStorage<Traits>(*rig.state);
  for(std::size_t first:{0u,63u,64u,65u,128u})for(bool status:{false,true}) {
    family.status[first]=status?17:0;family.result_valid[first]=status?1:0;
    auto result=rig.Compare();ASSERT_FALSE(::testing::Test::HasFailure());EXPECT_EQ(result.family,Traits::family);
    EXPECT_EQ(result.parent,first);EXPECT_EQ(result.status,status?s::BatchStatus::ElementFailure:s::BatchStatus::NonfiniteResult);
    if(index<4)EXPECT_EQ(result.diagnostics.parent_count[index+1],92u+index);
    family.status[first]=0;family.result_valid[first]=1;
  }
}
TEST(SolidMeasurementReadTileCuda, FamilyRowStatusAndInvalidResultKeepExactPrefix) {
  BadRows<b::Traits18>(0);BadRows<b::Traits24>(1);BadRows<b::Traits6z>(2);BadRows<b::Traits18Law44>(3);BadRows<b::Traits18Law90>(4);
}
TEST(SolidMeasurementReadTileCuda, FailedRowsAndUnvisitedFamiliesNeedNoPayload) {
  Rig rig(1);ASSERT_FALSE(HasFailure());auto& first=rig.state->solid18;
  first.status[0]=23;first.result_valid=nullptr;first.measurement=nullptr;
  rig.state->solid24.status=nullptr;rig.state->solid24.result_valid=nullptr;rig.state->solid24.measurement=nullptr;
  EXPECT_EQ(rig.Compare().element_status,23);ASSERT_FALSE(HasFailure());
  first.status[0]=0;auto* valid=rig.Allocate<std::uint8_t>(1);first.result_valid=valid;*valid=0;
  EXPECT_EQ(rig.Compare().status,s::BatchStatus::NonfiniteResult);ASSERT_FALSE(HasFailure());
}
TEST(SolidMeasurementReadTileCuda, PerParentFiniteFailurePrecedesLaterStatusAndFamilies) {
  Rig rig(129);ASSERT_FALSE(HasFailure());auto& first=rig.state->solid18;
  first.measurement[63].work=std::numeric_limits<double>::max();first.measurement[64].work=std::numeric_limits<double>::max();
  first.status[128]=31;rig.state->solid24.status[0]=37;
  const auto result=rig.Compare(false,Seed(true,0.));ASSERT_FALSE(HasFailure());
  EXPECT_EQ(result.status,s::BatchStatus::NonfiniteResult);EXPECT_EQ(result.family,s::Family::Solid18);EXPECT_EQ(result.parent,64u);
  EXPECT_EQ(result.diagnostics.parent_count[1],92u);
  first.measurement[63].work=std::nan("31");const auto nan=rig.Compare();ASSERT_FALSE(HasFailure());EXPECT_EQ(nan.parent,63u);
}
TEST(SolidMeasurementReadTileCuda, CancellationLocalOrderAndInitialWorkMask) {
  Rig rig(129);ASSERT_FALSE(HasFailure());auto& rows=rig.state->solid6z;
  for(unsigned p=0;p<129;++p){rows.measurement[p].work=0;for(unsigned n=0;n<6;++n)rows.measurement[p].kick[n]=rows.measurement[p].drift[n]=0;}
  const double terms[]{0x1p54,1,-0x1p54};
  for(unsigned i=0;i<3;++i){rows.measurement[63+i].work=terms[i];rows.measurement[63+i].kick[5]=terms[i];rows.measurement[63+i].drift[0]=terms[i];}
  for(double incoming:{0.,-0.,0x1p54,-0x1p54,std::numeric_limits<double>::denorm_min()}) {
    rig.Compare(false,Seed(true,incoming));ASSERT_FALSE(HasFailure());
  }
  rows.measurement[0].kick[0]=rows.measurement[0].drift[0]=std::nan("43");
  EXPECT_EQ(rig.Compare(true).status,s::BatchStatus::Success);ASSERT_FALSE(HasFailure());
  EXPECT_EQ(rig.Compare(false).status,s::BatchStatus::NonfiniteResult);ASSERT_FALSE(HasFailure());
}
TEST(SolidMeasurementReadTileCuda, SameDeviceRepairPublishesCompleteControl) {
  Rig rig(65);ASSERT_FALSE(HasFailure());auto& family=rig.state->solid18_law90;
  family.status[64]=41;auto failed=rig.Compare();ASSERT_FALSE(HasFailure());EXPECT_EQ(failed.element_status,41);
  family.status[64]=0;
  b::read_tile_current::Finalize<<<1,tile::Threads>>>(rig.state,0,1,{},Seed(false,.25),false,true);Rig::Drain();
  const auto repaired=rig.state->control;ASSERT_EQ(repaired.status,s::BatchStatus::Success);EXPECT_TRUE(repaired.diagnostics.valid);
  Frozen<<<1,1>>>(rig.state,{},Seed(false,.25),false);Rig::Drain();Same(repaired,rig.state->control);
}
} // namespace solid_read_tile_test
