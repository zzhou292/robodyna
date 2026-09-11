// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <algorithm>

namespace extended_solid_test {
namespace {
fe::SolidNodeContributions Ephemeral() {
  Fixture fixture;
  auto domain = fixture.Domain();
  fe::SolidNodeContributions result;
  const auto report = result.Initialize(domain, fixture.Input());
  EXPECT_TRUE(report) << report.message;
  return result; // All input references, node vectors and the caller handle die.
}
void SameParent(const fe::SolidCoefficientParent& a, const fe::SolidCoefficientParent& b) {
  EXPECT_EQ(a.family, b.family);
  EXPECT_EQ(a.source_element_id, b.source_element_id);
  EXPECT_EQ(a.source_part_id, b.source_part_id);
  EXPECT_EQ(a.source_section_id, b.source_section_id);
  EXPECT_EQ(a.source_material_id, b.source_material_id);
  ASSERT_EQ(a.node_count, b.node_count);
  for (unsigned slot = 0; slot < 8; ++slot) {
    EXPECT_EQ(a.source_node_id[slot], b.source_node_id[slot]);
    EXPECT_EQ(a.domain_node[slot], b.domain_node[slot]);
    EXPECT_EQ(coefficient_test::Bits(a.mass_kg[slot]), coefficient_test::Bits(b.mass_kg[slot]));
  }
}
}
TEST(ExtendedSolidCoefficients, DestroyedBorrowedInputsAndIndependentDomainKeepExactSnapshot) {
  const auto retained = Ephemeral();
  ASSERT_TRUE(retained.prepared());
  Fixture fixture;
  const auto domain = fixture.Domain();
  fe::SolidNodeContributions independent;
  ASSERT_TRUE(independent.Initialize(domain, fixture.Input()));
  EXPECT_TRUE(retained.Matches(independent));
  ASSERT_EQ(retained.parents().size(), 5u);
  for (unsigned parent = 0; parent < 5; ++parent) SameParent(retained.parents()[parent], independent.parents()[parent]);
  const auto copy = retained;
  EXPECT_TRUE(copy.Matches(retained));
  EXPECT_EQ(copy.parents().data(), retained.parents().data());
  const auto legacy = fixture.Solids(domain);
  for (unsigned parent = 0; parent < 3; ++parent) SameParent(retained.parents()[parent], legacy.parents()[parent]);
  EXPECT_FALSE(retained.Matches(legacy));
}
TEST(ExtendedSolidCoefficients, SingleExtendedFamilyAndReorderedDomainRetainTypedSourceSlots) {
  Fixture fixture;
  std::reverse(fixture.nodes.begin(), fixture.nodes.end());
  const auto domain = fixture.Domain();
  for (const bool rear : {false, true}) {
    auto input = fixture.Input();
    input.solid18 = nullptr; input.solid18_count = 0;
    input.solid24 = nullptr; input.solid24_count = 0;
    input.solid6z = nullptr; input.solid6z_count = 0;
    if (rear) { input.law90 = nullptr; input.law90_count = 0; }
    else { input.law44 = nullptr; input.law44_count = 0; }
    fe::SolidNodeContributions result;
    ASSERT_TRUE(result.Initialize(domain, input));
    ASSERT_EQ(result.parents().size(), 1u);
    const auto& parent = result.parents()[0];
    EXPECT_EQ(parent.family, rear ? Family::Solid18Law44 : Family::Solid18Law90);
    EXPECT_EQ(parent.node_count, 8u);
    const auto& source = rear ? fixture.rear_input : fixture.foam_input;
    const auto* masses = rear ? fixture.rear.mass().source_nodal_mass_kg : fixture.foam.mass().source_nodal_mass_kg;
    for (unsigned slot = 0; slot < 8; ++slot) {
      EXPECT_EQ(parent.domain_node[slot], domain.Find(source.source_node_id[slot]));
      EXPECT_EQ(parent.source_node_id[slot], source.source_node_id[slot]);
      EXPECT_EQ(coefficient_test::Bits(parent.mass_kg[slot]), coefficient_test::Bits(masses[slot]));
    }
    EXPECT_EQ(coefficient_test::Bits(parent.isotropic_inertia_kg_m2()), coefficient_test::Bits(0.0));
  }
}
} // namespace extended_solid_test
