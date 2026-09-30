#include "WeightTestSupport.h"
#include "lib_utest/qualification/surface_contact/NodalWallCapacitySource.h"

namespace vehicle_wall_weight_test {
TEST(VehicleWallWeights, MixedIndexedWeightsPreserveEveryLegacyBitAndInputOrderIndependence) {
  using namespace nodal_wall_capacity_test;
  auto source=std::make_unique<Source>();ASSERT_TRUE(source->Prepare());
  sc::NodalWallWeights legacy,indexed,reversed;
  ASSERT_EQ(legacy.Initialize(Nodes,source->input.data(),Parents,{}).status,sc::NodalWallStatus::Ok);
  ASSERT_EQ(indexed.Initialize(Nodes,source->input.data(),Parents,sc::NodalWallWeightLimits::Vehicle()).status,sc::NodalWallStatus::Ok);
  SameWeights(legacy,indexed);
  std::reverse(source->input.begin(),source->input.end());
  ASSERT_EQ(reversed.Initialize(Nodes,source->input.data(),Parents,sc::NodalWallWeightLimits::Vehicle()).status,sc::NodalWallStatus::Ok);
  source.reset();SameWeights(legacy,reversed);sc::NodalWallWeights copied(indexed),moved(std::move(copied));
  SameWeights(indexed,copied);SameWeights(indexed,moved);
}
TEST(VehicleWallWeights, DuplicateFacesFeaturesAndReferencePrecedenceMatchLegacy) {
  TriangleSource source(12,4);ASSERT_TRUE(source.Prepare());
  for(unsigned fault=0;fault<3;++fault) {
    auto parent=source.references[3].parent();
    if(fault==0)parent.parent_element_id=source.references[1].parent().parent_element_id;
    if(fault==1)parent.feature_id=source.references[0].parent().feature_id;
    sc::T3MaterialMeasure changed;
    ASSERT_EQ(sc::PrepareT3MaterialMeasure(source.View(),parent,&changed),sc::SurfaceMeasureStatus::Ok);
    const auto old=source.input[3];source.input[3]={nullptr,0,&changed};
    if(fault==2)source.input[3]={};
    sc::NodalWallWeights legacy,indexed;
    const auto a=legacy.Initialize(source.nodes,source.input.data(),source.input.size(),{});
    const auto b=indexed.Initialize(source.nodes,source.input.data(),source.input.size(),sc::NodalWallWeightLimits::Vehicle());
    EXPECT_EQ(a.status,fault==2?sc::NodalWallStatus::InvalidReference:sc::NodalWallStatus::DuplicateParent);
    EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.cause,b.cause);EXPECT_EQ(a.parent,b.parent);EXPECT_EQ(a.node,b.node);
    EXPECT_FALSE(indexed.prepared());source.input[3]=old;
  }
}
TEST(VehicleWallWeights, ExplicitProfileAndByteBoundsRejectBeforeBorrowedReads) {
  TriangleSource source(12,4);ASSERT_TRUE(source.Prepare());sc::NodalWallWeights value;
  auto limits=sc::NodalWallWeightLimits::Vehicle();
  ASSERT_EQ(value.Initialize(12,source.input.data(),4,limits).status,sc::NodalWallStatus::Ok);
  const auto kept=value;const auto* unreadable=reinterpret_cast<const sc::NodalWallParentInput*>(std::uintptr_t{1});
  const auto* retained=&value.parent(0);
  for(unsigned fault=0;fault<6;++fault) {
    auto cap=limits;
    if(fault==0)cap.max_owned_bytes=value.owned_payload_bytes()-1;
    if(fault==1)cap.max_startup_bytes=sc::nodal_wall_detail::WeightStartupBytes(4,12)-1;
    if(fault==2)cap.max_parents=3;
    if(fault==3)cap.max_nodes=11;
    if(fault==4) {cap={};cap.max_nodes=sc::MaxNodalWallWeightNodes+1;}
    if(fault==5)cap.profile=static_cast<sc::NodalWallWeightProfile>(99);
    const auto r=value.Initialize(12,unreadable,4,cap);
    EXPECT_EQ(r.status,fault<4?sc::NodalWallStatus::Capacity:sc::NodalWallStatus::InvalidInput)<<fault;
    EXPECT_EQ(&value.parent(0),retained);SameWeights(value,kept);
  }
  limits.max_owned_bytes=value.owned_payload_bytes();
  limits.max_startup_bytes=sc::nodal_wall_detail::WeightStartupBytes(4,12);
  ASSERT_EQ(value.Initialize(12,source.input.data(),4,limits).status,sc::NodalWallStatus::Ok);SameWeights(value,kept);
}
} // namespace vehicle_wall_weight_test
