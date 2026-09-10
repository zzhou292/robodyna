#include "Law1ForceTestSupport.h"
namespace layered_law1_force_test {
TEST(LayeredLaw1ForceValues,QephElasticHistoryThicknessWorkAndMismatchedSidecarRollback) {
  auto input=qeph_startup_test::Case(5);const auto material=Material(input);
  q::ReferenceData r;ASSERT_EQ(q::InitializeReference(input,r),q::Status::kSuccess);
  q::LayeredLaw1History base;ASSERT_EQ(q::InitializeLayeredLaw1History(r,material,{},base),q::Status::kSuccess);
  path::Path motion{false,1};q::LayeredLaw1ForceTrial value;
  const auto in=motion.Interval(input);
  ASSERT_EQ(q::EvaluateLayeredLaw1Force(r,material,base,in,value),q::Status::kSuccess);
  EXPECT_NE(value.force.proposed_history.data().thickness,input.thickness);
  EXPECT_GT(value.force.proposed_history.data().internal_work[0],0.);
  EXPECT_GT(std::abs(value.proposed_section.point[0].stress[0]-value.proposed_section.point[2].stress[0]),1.);
  const auto before=Bytes(value);const auto accepted=Bytes(base);
  auto bad=base;bad.section.point[2].stress[0]=1.;
  EXPECT_EQ(q::EvaluateLayeredLaw1Force(r,material,bad,in,value),q::Status::kInvalidInput);
  EXPECT_EQ(Bytes(value),before);EXPECT_EQ(Bytes(base),accepted);
  ASSERT_EQ(q::EvaluateLayeredLaw1Force(r,material,base,in,value),q::Status::kSuccess);
  EXPECT_EQ(Bytes(value),before);
}
TEST(LayeredLaw1ForceValues,T3ElasticHistoryThicknessWorkAndWrongMaterialRollback) {
  auto input=t3_port_test::Triangle(.02,1);const auto material=Material(input,10e9);
  t::ReferenceData r;ASSERT_EQ(t::InitializeReference(input,r),t::Status::kSuccess);
  t::LayeredLaw1History base;ASSERT_EQ(t::InitializeLayeredLaw1History(r,material,{},base),t::Status::kSuccess);
  path::Path motion{false,1};t::LayeredLaw1ForceTrial value;const auto in=motion.Interval(input);
  ASSERT_EQ(t::EvaluateLayeredLaw1Force(r,material,base,in,value),t::Status::kSuccess);
  EXPECT_NE(value.force.proposed_history.data().thickness,input.thickness);
  EXPECT_GT(value.force.proposed_history.data().internal_work[0],0.);
  const auto before=Bytes(value);auto bad=material;bad.young_pa*=2;
  EXPECT_EQ(t::EvaluateLayeredLaw1Force(r,bad,base,in,value),t::Status::kInvalidInput);
  EXPECT_EQ(Bytes(value),before);
  auto phase=in;++phase.sample_index;
  EXPECT_EQ(t::EvaluateLayeredLaw1Force(r,material,base,phase,value),t::Status::kInvalidInput);
  EXPECT_EQ(Bytes(value),before);
  ASSERT_EQ(t::EvaluateLayeredLaw1Force(r,material,base,in,value),t::Status::kSuccess);
  EXPECT_EQ(Bytes(value),before);
}
} // namespace layered_law1_force_test
