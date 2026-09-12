// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Source.h"
#include "lib_src/elements/beam18/resident/Arena.h"
#include "lib_src/elements/publication/PhysicalState.h"
namespace beam18_publication_test {
TEST(BeamPublicationValues, CompleteV5UsesSameSixProducerSourcesAndFullRigidMembers) {
  existing::Fixture base; Source source;
  ASSERT_TRUE(source.Initialize(base));
  EXPECT_TRUE(source.model.domain()->SharesStorage(base.domain));
  EXPECT_TRUE(source.physical.coefficients()->Matches(source.ledger));
  EXPECT_TRUE(source.ledger.solids()->Matches(*base.solids.contributions()));
  EXPECT_TRUE(source.ledger.type13()->Matches(base.beam_coefficients));
  EXPECT_EQ(source.model.parents().size(),3u);
  EXPECT_EQ(source.ledger.scope().uncovered_nodes,0u);
  EXPECT_EQ(source.rigid.groups().size(),base.rigid.groups().size());
  EXPECT_EQ(source.rigid.members().size(),base.rigid.members().size());
  EXPECT_TRUE(source.rigid.FindMember(source.model.parents()[0].domain_nodes[0]));
  EXPECT_TRUE(source.rigid.FindMember(source.model.parents()[1].domain_nodes[0]));
  EXPECT_FALSE(source.rigid.FindMember(source.model.parents()[2].domain_nodes[0]));
  const auto& member=source.plain.members()[0];
  EXPECT_EQ(member.unpartitioned_native_inertia_kg_m2,
      source.ledger.nodes()[member.global_node].coefficients.beam18.isotropic_inertia);
  EXPECT_GT(member.unpartitioned_native_inertia_kg_m2,0);
}
TEST(BeamPublicationValues, OwnedCurveLifetimeAndAllBeamSourceMetadataRemainExact) {
  existing::Fixture base; Source source;
  ASSERT_TRUE(source.Initialize(base));
  const auto& material=source.model.materials()[0];
  EXPECT_EQ(material.source_material_id,32300u); EXPECT_EQ(material.value.curve.count,46u);
  b::Material prepared;
  ASSERT_EQ(b::point::Prepare(material.value.material,material.value.curve,prepared),b::point::Status::Ok);
  EXPECT_TRUE(b::force_detail::SameMaterial(prepared,material.value));
  for (unsigned p=0;p<3;++p) {
    const auto& parent=source.model.parents()[p];
    EXPECT_EQ(parent.reference.input().source_element_id,32000u+p);
    EXPECT_EQ(parent.reference.input().source_node_id[2],0u);
    EXPECT_EQ(parent.material_index,0u);
  }
}
} // namespace beam18_publication_test
