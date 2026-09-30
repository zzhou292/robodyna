// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SolidSupport.h"
#include "lib_utest/qualification/solid_candidate_validation/CudaRig.h"
namespace tl::fea::solids::batch_detail {
void LaunchControlFinalizeTest(Storage*,unsigned,unsigned,NodalPreparedView,BatchDiagnostics,bool,bool);
}
namespace final_control_test::solid {
using solid_validation_test::DeviceRig;
__global__ void FrozenFinalize(b::Storage* state,fe::NodalPreparedView view,s::BatchDiagnostics identity,bool initial,bool operands) {
  b::control2733::Finalize(state,0,1,view,identity,initial,operands);
}
inline void CompareDevice(DeviceRig& rig,s::BatchDiagnostics identity,bool initial,bool operands) {
  auto view=rig.View(0);ASSERT_FALSE(::testing::Test::HasFailure());
  ASSERT_TRUE(rig.Restore(rig.before));
  b::LaunchMeasurementValidation(rig.device,0,1,view,identity.time,identity.epoch,initial,rig.stream);
  std::vector<unsigned char> staged;ASSERT_TRUE(rig.Read(staged));
  b::LaunchControlFinalizeTest(rig.device,0,1,view,identity,initial,operands);
  ASSERT_TRUE(rig.Read(rig.parallel));ASSERT_TRUE(rig.Restore(staged));
  FrozenFinalize<<<1,1,0,rig.stream>>>(rig.device,view,identity,initial,operands);
  ASSERT_TRUE(rig.Read(rig.serial));rig.Compare();
  const auto offset=offsetof(b::Storage,control),end=offset+sizeof(b::Control);
  EXPECT_EQ(std::memcmp(staged.data(),rig.parallel.data(),offset),0);
  EXPECT_EQ(std::memcmp(staged.data()+end,rig.parallel.data()+end,staged.size()-end),0);
}
TEST(SolidLocalControlCuda, RealAllFamilyArenasPreserveCompleteSeededControlOnBothRoutes) {
  DeviceRig rig;ASSERT_TRUE(rig.Initialize());const auto original=rig.serial;
  const b::FamilyLayout* layouts[]{&rig.host.layout.solid18,&rig.host.layout.solid24,&rig.host.layout.solid6z,
      &rig.host.layout.solid18_law44,&rig.host.layout.solid18_law90};
  for(int fault=-1;fault<5;++fault)for(bool initial:{false,true})for(bool operands:{false,true})for(bool valid:{false,true}) {
    SCOPED_TRACE(::testing::Message()<<fault<<":"<<initial<<":"<<operands<<":"<<valid);
    rig.before=original;reinterpret_cast<b::Storage*>(rig.before.data())->control=Poison();
    if(fault>=0)*tl::util::ArenaPointer<int>(rig.before.data(),layouts[fault]->status)=17;
    CompareDevice(rig,Seed(valid,0x1p54),initial,operands);ASSERT_FALSE(HasFailure());
    EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status,fault<0?s::BatchStatus::Success:s::BatchStatus::ElementFailure);
    if(fault>=0)EXPECT_EQ(DeviceRig::Header(rig.parallel).control.diagnostics.valid,valid);
  }
}
TEST(SolidLocalControlCuda, CrossFamilyOverflowKeepsEarlierFailureAndUnvisitedSeedCounters) {
  DeviceRig rig;rig.host.source.Repeat44();ASSERT_TRUE(rig.Initialize());const auto original=rig.serial;
  for(bool initial:{false,true})for(bool operands:{false,true}) {
    rig.before=original;reinterpret_cast<b::Storage*>(rig.before.data())->control=Poison();
    auto* rows=tl::util::ArenaPointer<b::State<b::Traits18Law44>>(rig.before.data(),rig.host.layout.solid18_law44.slab[1]);
    rows[0].cache.diagnostics.internal_work_increment_j=DBL_MAX;
    rows[1].cache.diagnostics.internal_work_increment_j=DBL_MAX;
    *tl::util::ArenaPointer<int>(rig.before.data(),rig.host.layout.solid18_law90.status)=19;
    CompareDevice(rig,Seed(true,0.),initial,operands);ASSERT_FALSE(HasFailure());
    const auto& control=DeviceRig::Header(rig.parallel).control;
    EXPECT_EQ(control.status,s::BatchStatus::NonfiniteResult);EXPECT_EQ(control.family,s::Family::Solid18Law44);
    EXPECT_EQ(control.parent,1u);EXPECT_EQ(control.diagnostics.parent_count[4],95u);EXPECT_TRUE(control.diagnostics.valid);
  }
}
TEST(SolidLocalControlCuda, SameDeviceRepairKeepsAcceptedAndTrialHistoryUnchanged) {
  DeviceRig rig;ASSERT_TRUE(rig.Initialize());rig.before=rig.serial;
  *tl::util::ArenaPointer<int>(rig.before.data(),rig.host.layout.solid18_law90.status)=23;
  auto identity=Seed(false,.25);auto view=rig.View(0);ASSERT_TRUE(rig.Restore(rig.before));
  b::LaunchMeasurementValidation(rig.device,0,1,view,identity.time,identity.epoch,false,rig.stream);
  b::LaunchControlFinalizeTest(rig.device,0,1,view,identity,false,true);ASSERT_TRUE(rig.Read(rig.parallel));
  ASSERT_EQ(DeviceRig::Header(rig.parallel).control.status,s::BatchStatus::ElementFailure);
  const int repaired=0;
  auto* status=reinterpret_cast<unsigned char*>(rig.device)+rig.host.layout.solid18_law90.status.offset;
  ASSERT_EQ(cudaMemcpyAsync(status,&repaired,sizeof(repaired),cudaMemcpyHostToDevice,rig.stream),cudaSuccess);
  b::LaunchMeasurementValidation(rig.device,0,1,view,identity.time,identity.epoch,false,rig.stream);
  b::LaunchControlFinalizeTest(rig.device,0,1,view,identity,false,true);ASSERT_TRUE(rig.Read(rig.parallel));
  ASSERT_EQ(DeviceRig::Header(rig.parallel).control.status,s::BatchStatus::Success);
  FrozenFinalize<<<1,1,0,rig.stream>>>(rig.device,view,identity,false,true);ASSERT_TRUE(rig.Read(rig.serial));rig.Compare();
  const b::FamilyLayout* layouts[]{&rig.host.layout.solid18,&rig.host.layout.solid24,&rig.host.layout.solid6z,
      &rig.host.layout.solid18_law44,&rig.host.layout.solid18_law90};
  for(const auto* family:layouts)for(unsigned slab=0;slab<2;++slab)
    EXPECT_EQ(std::memcmp(rig.before.data()+family->slab[slab].offset,
        rig.parallel.data()+family->slab[slab].offset,family->slab[slab].bytes),0);
}
} // namespace final_control_test::solid
