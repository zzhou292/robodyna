#include "NodalWallOwnerFixture.h"

namespace {
using namespace nodal_wall_owner_test;
TEST(NodalWallModel, BindsWholeReferenceMeasureAndGlobalNodesWithoutAllocatingOwner) {
  Fixture f; f.n=Capacity; ASSERT_TRUE(f.Prepare()); detail::Model model;
  ASSERT_EQ(f.Model(&model,f.Config()).status,Code::Ok);
  EXPECT_TRUE(model.prepared); EXPECT_TRUE(model.query.prepared()); EXPECT_TRUE(model.coverage.covered);
  EXPECT_EQ(model.node_count,6u); EXPECT_EQ(model.parent_count,2u); EXPECT_EQ(model.config.owner.node_count,Capacity);
  Near(model.parents[0].area.value,.5L); Near(model.parents[1].area.value,1.L);
  Near(model.nodes[2].area.value,.375L); Near(model.rate,6.L);
  EXPECT_EQ(model.parents[0].nodes[0],2u); EXPECT_EQ(model.nodes[5].node,5u);
  EXPECT_LE(sizeof(detail::Storage),sc::MaxNodalWallDeviceBytes);
  RecordProperty("device_storage_bytes",std::to_string(sizeof(detail::Storage)));
  RecordProperty("immutable_model_bytes",std::to_string(sizeof(detail::Model)));
  RecordProperty("compact_result_bytes",std::to_string(sizeof(sc::NodalWallDeviceResults)));
  Fixture reordered; ASSERT_TRUE(reordered.Prepare(true)); detail::Model permuted;
  ASSERT_EQ(reordered.Model(&permuted,reordered.Config()).status,Code::Ok);
  for (unsigned p=0;p<2;++p) {
    EXPECT_EQ(model.parents[p].parent_element_id,permuted.parents[p].parent_element_id);
    Same(model.parents[p].area,permuted.parents[p].area); Same(model.parents[p].share,permuted.parents[p].share);
  }
}
TEST(NodalWallModel, InvalidIdentityMassAndCapsPreserveEveryPreparedByte) {
  Fixture f; ASSERT_TRUE(f.Prepare()); detail::Model out;
  ASSERT_EQ(f.Model(&out,f.Config()).status,Code::Ok); const auto before=Bytes(out);
  for (unsigned variant=0;variant<9;++variant) {
    SCOPED_TRACE(variant); auto c=f.Config();
    if (variant==0) c.owner.owner_id=0;
    if (variant==1) c.owner.temporal_scheme=fe::NodalTemporalScheme::VelocityFirst;
    if (variant==2) c.owner.epoch=1;
    if (variant==3) c.qualification_id=0;
    if (variant==4) c.max_device_bytes=sizeof(detail::Storage)-1;
    if (variant==5) c.law.parent_energy_error=0;
    if (variant==6) c.law.maximum_penetration=0;
    if (variant==7) c.law.stiffness_per_area=std::numeric_limits<double>::infinity();
    if (variant==8) c.owner.node_count=65;
    EXPECT_NE(f.Model(&out,c).status,Code::Ok); Unchanged(out,before);
  }
  f.fixed[5]=6; EXPECT_EQ(f.Model(&out,f.Config()).status,Code::InvalidMass); Unchanged(out,before);
  f.fixed[5]=0; f.inverse[5]=0;
  EXPECT_EQ(f.Model(&out,f.Config()).status,Code::InvalidMass); Unchanged(out,before);
  f.inverse[5]=1; ASSERT_EQ(f.Model(&out,f.Config()).status,Code::Ok);
  EXPECT_GE(out.rate,6.); Near(out.rate,6.L);
  sc::T3MaterialMeasure tri;
  const sc::SurfaceTriangle parent{{0,1,2},333,444,0,0,sc::SurfaceInterpolation::kLinearTriangle};
  ASSERT_EQ(sc::PrepareT3MaterialMeasure(f.View(f.reference),parent,&tri),sc::SurfaceMeasureStatus::Ok);
  sc::NodalWallWeights triangle_weights; const sc::NodalWallParentInput input{nullptr,0,&tri};
  ASSERT_EQ(triangle_weights.Initialize(f.n,&input,1).status,sc::NodalWallStatus::Ok);
  const auto held=Bytes(out);
  f.inverse[2]=0;
  EXPECT_EQ(detail::PrepareModel(f.Config(),f.wall.view(),triangle_weights,f.View(f.x),f.inverse.data(),
      f.fixed.data(),f.motion,&out).status,Code::InvalidMass); Unchanged(out,held);
  f.inverse[2]=1;
  ASSERT_EQ(detail::PrepareModel(f.Config(),f.wall.view(),triangle_weights,f.View(f.x),f.inverse.data(),
      f.fixed.data(),f.motion,&out).status,Code::Ok);
  EXPECT_EQ(out.parents[0].family,sc::NodalWallParentFamily::T3Native);
  EXPECT_EQ(out.parents[0].arity,3u);
}
TEST(NodalWallModel, ActualFiniteEnvelopeRejectsHolesAndOutsideWithoutPointSubstitution) {
  Fixture f; ASSERT_TRUE(f.Prepare()); detail::Model out;
  ASSERT_EQ(f.Model(&out,f.Config()).status,Code::Ok); const auto before=Bytes(out);
  f.wall=q4_planar_test::Ring(); EXPECT_EQ(f.Model(&out,f.Config()).status,Code::GeometryFailure); Unchanged(out,before);
  f.wall=q4_planar_test::Square(); f.motion.maximum.y=3;
  EXPECT_EQ(f.Model(&out,f.Config()).status,Code::GeometryFailure); Unchanged(out,before);
  f.motion.maximum.y=1; f.wall.vertices.back().position.y=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(f.Model(&out,f.Config()).status,Code::GeometryFailure); Unchanged(out,before);
  f.wall=q4_planar_test::Square(2); ASSERT_EQ(f.Model(&out,f.Config()).status,Code::Ok);
  EXPECT_EQ(out.query.face_count(),8u); Near(out.rate,6.L);
}
TEST(NodalWallModel, FixedNodeMustBeNonpenetratingAndSourceTupleKeepsErrorBudgets) {
  Fixture f; f.fixed[0]=7; f.inverse[0]=0; f.x[0]=0; ASSERT_TRUE(f.Prepare()); detail::Model out;
  ASSERT_EQ(f.Model(&out,f.Config()).status,Code::Ok); EXPECT_EQ(out.fixed[0],1u);
  const auto before=Bytes(out); f.x[0]=std::numeric_limits<double>::denorm_min();
  EXPECT_NE(f.Model(&out,f.Config()).status,Code::Ok); Unchanged(out,before);
  Fixture source; source.Depth(.00025); ASSERT_TRUE(source.Prepare()); auto c=source.Config();
  c.law.stiffness_per_area=4e5; c.law.maximum_penetration=.0005;
  ASSERT_EQ(source.Model(&out,c).status,Code::Ok);
  EXPECT_EQ(out.config.law.parent_force_error,ForceBudget); EXPECT_EQ(out.config.law.parent_energy_error,EnergyBudget);
  EXPECT_GE(out.rate,150000.); Near(out.rate,150000.L);
  source.x[15]=std::nextafter(.0005,std::numeric_limits<double>::infinity());
  EXPECT_EQ(source.Model(&out,c).status,Code::PointFailure);
}
} // namespace
