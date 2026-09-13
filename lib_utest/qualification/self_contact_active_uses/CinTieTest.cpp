// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace active_use_test {
TEST(SelfContactActiveUses, CinSecondaryRejectsMasterAdmitsAndLocalTieRemainsUnresolved) {
  Fixture fixture(2,false,true);
  Tie tie(fixture);
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets,{nullptr,tie.Source()}).status,Code::Ok);
  ASSERT_EQ(uses.cin().witness_count,1u);
  auto active = fixture.Active(uses);
  const c::SelfContactActivityView view{active.data(),active.data(),active.size()};

  const auto secondary = fixture.VertexUse(101,uses,20);
  ASSERT_NE(secondary,SIZE_MAX);
  const auto master_face = fixture.RemoteFacet(100,
      uses.vertex_uses()[secondary].feature,uses);
  ASSERT_NE(master_face,SIZE_MAX);
  c::SelfContactPairClassification pair;
  ASSERT_EQ(uses.ClassifyVertexFace(secondary,master_face,
      fixture.FacePoint(master_face,uses),view,&pair).status,Code::Ok);
  EXPECT_EQ(pair.endpoint_support[0].status,
      c::SelfContactSupportStatus::UnsupportedCinSecondary);
  EXPECT_EQ(pair.status,c::SelfContactPairStatus::UnsupportedCinSecondary);
  EXPECT_EQ(pair.tied,
      c::SelfContactTiedStatus::CompleteLocalSupportNeedsRuntimeActivity);
  EXPECT_FALSE(pair.excluded);

  const auto unrelated = fixture.VertexUse(101,uses,21);
  ASSERT_NE(unrelated,SIZE_MAX);
  ASSERT_EQ(uses.ClassifyVertexFace(unrelated,master_face,
      fixture.FacePoint(master_face,uses),view,&pair).status,Code::Ok);
  EXPECT_EQ(pair.tied,c::SelfContactTiedStatus::NotRelated);
  EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);

  const auto master = fixture.VertexUse(100,uses,10);
  ASSERT_NE(master,SIZE_MAX);
  std::size_t independent_face = SIZE_MAX, independent_slot = SIZE_MAX;
  const auto independent_parent = fixture.Parent(101,uses);
  const auto& parent = uses.parents()[independent_parent];
  for (std::size_t f = parent.facet_offset;
       f < std::size_t(parent.facet_offset)+parent.facet_count; ++f) {
    for (unsigned slot = 0; slot < 3; ++slot) {
      const auto& point = uses.vertex_uses()[uses.facet_uses()[f].vertex_uses[slot]].point;
      unsigned nonzero = 0;
      std::uint64_t source_id = 0;
      for (unsigned n = 0; n < point.count; ++n)
        if (point.weights[n] != 0) {
          ++nonzero;
          source_id = fixture.domain.nodes()[point.nodes[n]].source_id;
        }
      if (nonzero == 1 && source_id == 21) {
        independent_face = f; independent_slot = slot;
      }
    }
    if (independent_face != SIZE_MAX) break;
  }
  ASSERT_NE(independent_face,SIZE_MAX);
  const auto face_point = uses.vertex_uses()[
      uses.facet_uses()[independent_face].vertex_uses[independent_slot]].point;
  ASSERT_EQ(uses.ClassifyVertexFace(master,independent_face,face_point,
      view,&pair).status,Code::Ok);
  EXPECT_EQ(pair.endpoint_support[0].status,
      c::SelfContactSupportStatus::AdmittedCinMaster);
  EXPECT_EQ(pair.status,c::SelfContactPairStatus::AdmittedVertexFace);
}

TEST(SelfContactActiveUses, IncompleteOrUnrelatedWitnessNeverInventsTieExclusion) {
  Fixture fixture(2,false,true);
  Tie tie(fixture);
  tie.witnesses[0].source_element_id = 101;
  c::SelfContactActiveUseBinding uses;
  ASSERT_EQ(uses.Initialize(fixture.facets,{nullptr,tie.Source()}).status,Code::Ok);
  auto active = fixture.Active(uses);
  const auto secondary = fixture.VertexUse(101,uses,20);
  const auto face = fixture.RemoteFacet(100,
      uses.vertex_uses()[secondary].feature,uses);
  c::SelfContactPairClassification pair;
  ASSERT_EQ(uses.ClassifyVertexFace(secondary,face,fixture.FacePoint(face,uses),
      {active.data(),active.data(),active.size()},&pair).status,Code::Ok);
  EXPECT_EQ(pair.tied,
      c::SelfContactTiedStatus::PartialOrUnauthenticatedLocalSupportNotExcluded);
  EXPECT_FALSE(pair.excluded);
}
} // namespace active_use_test
