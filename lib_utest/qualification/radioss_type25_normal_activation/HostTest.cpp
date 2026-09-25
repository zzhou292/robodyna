// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <algorithm>
namespace normal_activation_test {
TEST(NormalActivation, CompleteMasksMatchOriginalFreeRosterAndTaggingBranches) {
  for(unsigned scenario=0;scenario<12;++scenario) {
    SCOPED_TRACE(scenario);auto f=Scenario(scenario);const auto native=Oracle(f.Input());
    std::vector<std::uint32_t> mains(f.scene.mains.size(),99),nodes(f.scene.nodes.size(),88);
    ASSERT_EQ(a::EvaluateNativeNormalActivation(f.Input(),Fixture::Limits(),
      {mains.data(),mains.size(),nodes.data(),nodes.size()}),a::Status::Ok);
    EXPECT_EQ(mains,native.main_active);EXPECT_EQ(nodes,native.node_tag);EXPECT_EQ(f.free,native.free_main_ids);
    for(auto value:mains)EXPECT_LE(value,1u);for(auto value:nodes)EXPECT_LE(value,1u);
    if(scenario==2){EXPECT_EQ(mains[0],1u);EXPECT_TRUE(std::all_of(nodes.begin(),nodes.end(),[](auto x){return x==0;}));}
    if(scenario==9)EXPECT_TRUE(f.free.empty());
  }
}
TEST(NormalActivation, StaleOrIncompleteFreeRostersRejectBeforeAnyMaskWrite) {
  auto f=Scenario(7);ASSERT_EQ(f.free.size(),2u);
  for(unsigned fault=0;fault<5;++fault) {
    auto broken=f;auto in=broken.Input();
    if(fault==0){broken.free.pop_back();in=broken.Input();}
    if(fault==1){std::reverse(broken.free.begin(),broken.free.end());in=broken.Input();}
    if(fault==2){broken.free[1]=broken.free[0];in=broken.Input();}
    if(fault==3)in.profile.free_roster=a::FreeRosterPolicy::Unspecified;
    if(fault==4){broken.scene.mains[0].coefficient=0;in=broken.Input();}
    std::vector<std::uint32_t> mains(3,17),nodes(7,29);
    EXPECT_NE(a::EvaluateNativeNormalActivation(in,Fixture::Limits(),{mains.data(),3,nodes.data(),7}),a::Status::Ok);
    EXPECT_EQ(mains,std::vector<std::uint32_t>(3,17));EXPECT_EQ(nodes,std::vector<std::uint32_t>(7,29));
  }
}
TEST(NormalActivation, NoFloatNormalReadAndOptimizedIntegerUnionIsOrderIndependent) {
  auto f=Scenario(6);const auto expected=Oracle(f.Input());
  for(auto& main:f.scene.mains)for(auto& value:main.normal_slot)value.x=std::numeric_limits<float>::quiet_NaN();
  for(auto& ref:f.scene.normals)for(auto& value:ref.bisector)value.y=std::numeric_limits<float>::infinity();
  std::reverse(f.optimized.begin(),f.optimized.end());
  std::vector<std::uint32_t> mains(3),nodes(7);
  ASSERT_EQ(a::EvaluateNativeNormalActivation(f.Input(),Fixture::Limits(),{mains.data(),3,nodes.data(),7}),a::Status::Ok);
  EXPECT_EQ(mains,expected.main_active);EXPECT_EQ(nodes,expected.node_tag);
}
TEST(NormalActivation, InvalidStageCountsLateIdsCapsAndAliasPreserveOutputs) {
  for(unsigned fault=0;fault<9;++fault) {
    auto f=Scenario(6);auto in=f.Input();auto limits=Fixture::Limits();
    std::vector<std::uint32_t> mains(3,31),nodes(7,41);a::Output out{mains.data(),3,nodes.data(),7};
    if(fault==0)f.rows[0].stage.report.stage=l::Stage::Retained;
    if(fault==1)f.rows[0].stage.report.count_complete=true;
    if(fault==2)--f.rows[0].stage.value.optimized_count;
    if(fault==3)f.optimized.back()=4;
    if(fault==4)f.scene.mains.back().normal_reference[3]=65;
    if(fault==6)out.main_active=reinterpret_cast<std::uint32_t*>(f.scene.mains.data());
    if(fault==7)out.node_tag=mains.data();
    if(fault==8)f.rows[0].stage.value.history.generation++;
    // Existing pointers remain stable; no vector size changes in these faults.
    if(fault==5)limits.mains=2;
    EXPECT_NE(a::EvaluateNativeNormalActivation(in,limits,out),a::Status::Ok);
    EXPECT_EQ(mains,std::vector<std::uint32_t>(3,31));EXPECT_EQ(nodes,std::vector<std::uint32_t>(7,41));
    EXPECT_EQ(f.scene.mains[0].global_id,11);
  }
}
} // namespace normal_activation_test
