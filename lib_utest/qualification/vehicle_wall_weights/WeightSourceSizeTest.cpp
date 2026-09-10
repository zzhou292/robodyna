#include "WeightTestSupport.h"

namespace vehicle_wall_weight_test {
TEST(VehicleWallWeights, CompleteSourceCountExtentPreservesAllSharesAndLateFailureRetry) {
  constexpr std::uint32_t Nodes=359785,Parents=349645;
  // Authentic retained count shape, synthetic triangles/positions/identities.
  // This validates host weight capacity, not original vehicle contact geometry.
  auto source=std::make_unique<TriangleSource>(Nodes,Parents);ASSERT_TRUE(source->Prepare());
  sc::NodalWallWeights weights;
  EXPECT_EQ(weights.Initialize(Nodes,source->input.data(),Parents,{}).status,sc::NodalWallStatus::Capacity);
  auto limits=sc::NodalWallWeightLimits::Vehicle();
  ASSERT_EQ(weights.Initialize(Nodes,source->input.data(),Parents,limits).status,sc::NodalWallStatus::Ok);
  ASSERT_EQ(weights.parent_count(),Parents);ASSERT_EQ(weights.node_count(),Nodes);
  std::vector<std::uint32_t> incidence(Nodes);
  for(unsigned p=0;p<Parents;++p) {
    const auto& actual=weights.parent(p);
    ASSERT_EQ(actual.parent_element_id,TriangleSource::FirstId+2*p+1);ASSERT_EQ(actual.arity,3u);
    EXPECT_LE(actual.area.lower,.5);EXPECT_GE(actual.area.upper,.5);
    for(unsigned i=0;i<3;++i)++incidence[actual.nodes[i]];
  }
  for(unsigned n=0;n<Nodes;++n) {
    const auto& actual=weights.node(n);ASSERT_EQ(actual.node,n);ASSERT_GT(incidence[n],0u);
    const long double expected=incidence[n]/6.L;
    EXPECT_LE(actual.area.lower,expected);EXPECT_GE(actual.area.upper,expected);
  }
  EXPECT_LE(weights.total_area().lower,Parents/2.L);EXPECT_GE(weights.total_area().upper,Parents/2.L);
  const auto kept=weights;const auto* retained=&weights.parent(0);const auto original=source->input.back();
  source->input.back()={};const auto failed=weights.Initialize(Nodes,source->input.data(),Parents,limits);
  EXPECT_EQ(failed.status,sc::NodalWallStatus::InvalidReference);EXPECT_EQ(failed.parent,Parents-1);
  EXPECT_EQ(&weights.parent(0),retained);SameWeights(weights,kept);source->input.back()=original;
  limits.max_owned_bytes=weights.owned_payload_bytes();
  limits.max_startup_bytes=sc::nodal_wall_detail::WeightStartupBytes(Parents,Nodes);
  ASSERT_EQ(weights.Initialize(Nodes,source->input.data(),Parents,limits).status,sc::NodalWallStatus::Ok);
  source.reset();SameWeights(weights,kept);
  RecordProperty("retained_weight_bytes",std::to_string(weights.owned_payload_bytes()));
  RecordProperty("startup_scratch_bytes",std::to_string(limits.max_startup_bytes));
  RecordProperty("scope","source-count synthetic triangle weights; no resident contact or vehicle source admission");
}
} // namespace vehicle_wall_weight_test
