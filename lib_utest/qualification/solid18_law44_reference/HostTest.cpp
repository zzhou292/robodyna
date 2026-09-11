#include "TestSupport.h"
#include <gtest/gtest.h>
#include <limits>

namespace rear18_test {
TEST(Rear18Reference, ExplicitLaw44ProfilePreservesStrictLegacyEntryAndSharedGeometry) {
  auto input = Cube();
  law::Reference prepared;
  ASSERT_EQ(law::InitializeReference(input, prepared), s::Status::Success);
  EXPECT_EQ(prepared.topology(), law::SourceTopology::EightDistinct);
  s::Reference legacy;
  EXPECT_EQ(s::InitializeReference(input, legacy), s::Status::UnsupportedProfile);
  input.profile = {};
  EXPECT_EQ(law::InitializeReference(input, prepared), s::Status::UnsupportedProfile);
  ASSERT_EQ(s::InitializeReference(input, legacy), s::Status::Success);
  const auto before = Values(prepared), old = solid18_test::Values(legacy);
  EXPECT_EQ(std::memcmp(before.data(), old.data(), sizeof(before)), 0);
  EXPECT_DOUBLE_EQ(prepared.mass().element_mass_kg, input.density_kg_m3);
}
TEST(Rear18Reference, OriginalRepeatedSlotsKeepEightMassTermsAndSixSourceIdentities) {
  auto input = Collapsed();
  law::Reference prepared;
  ASSERT_EQ(law::InitializeReference(input, prepared), s::Status::Success);
  EXPECT_EQ(prepared.topology(), law::SourceTopology::RepeatedPairs56And78);
  EXPECT_NEAR(prepared.geometry().center_volume_m3, .5, 1e-14);
  EXPECT_NEAR(prepared.geometry().integrated_volume_m3, .5, 1e-14);
  EXPECT_NEAR(prepared.mass().element_mass_kg, .5*input.density_kg_m3, 1e-10);
  for (double value : prepared.mass().source_nodal_mass_kg)
    EXPECT_DOUBLE_EQ(value, prepared.mass().source_nodal_mass_kg[0]);
  const double first = prepared.mass().source_nodal_mass_kg[0];
  EXPECT_DOUBLE_EQ(prepared.mass().source_nodal_mass_kg[4] + prepared.mass().source_nodal_mass_kg[5], 2*first);
  EXPECT_NE(2*first, prepared.mass().element_mass_kg/6);
  s::Reference legacy;
  input.profile = {};
  EXPECT_EQ(s::InitializeReference(input, legacy), s::Status::InvalidInput);
}
TEST(Rear18Reference, NativeOrientationReversesOnlyInternalSlotPermutation) {
  for (bool collapsed : {false, true}) {
    auto input = collapsed ? Collapsed() : Cube();
    for (auto& point : input.position_m) point.x = -point.x;
    law::Reference result;
    ASSERT_EQ(law::InitializeReference(input, result), s::Status::Success);
    for (unsigned n = 0; n < 8; ++n) {
      EXPECT_EQ(result.source_slot(n), (n+4)%8);
      EXPECT_EQ(result.input().source_node_id[n], input.source_node_id[n]);
    }
    EXPECT_EQ(result.source_slot(8), 8u);
    for (const auto& point : result.geometry().point) EXPECT_GT(point.initial_volume_m3, 0);
  }
}
TEST(Rear18Reference, LateRepeatedIdentityAndCoordinateFailuresPreserveOutputAndRetry) {
  const auto original = Collapsed();
  law::Reference result;
  ASSERT_EQ(law::InitializeReference(original, result), s::Status::Success);
  const auto old = solid18_test::Bytes(result);
  const auto accepted_values = Values(result);
  auto bad = original;
  bad.source_node_id[7] = bad.source_node_id[0];
  EXPECT_EQ(law::InitializeReference(bad, result), s::Status::InvalidInput);
  EXPECT_EQ(solid18_test::Bytes(result), old);
  bad = original;
  bad.position_m[7].z = std::nextafter(bad.position_m[6].z, 2.0);
  EXPECT_EQ(law::InitializeReference(bad, result), s::Status::InvalidInput);
  EXPECT_EQ(solid18_test::Bytes(result), old);
  bad = original;
  bad.position_m[5].y = -0.0;
  EXPECT_EQ(law::InitializeReference(bad, result), s::Status::InvalidInput);
  EXPECT_EQ(solid18_test::Bytes(result), old);
  bad = original;
  bad.source_node_id[5] = 9999;  // Seven distinct nodes are outside this source profile.
  EXPECT_EQ(law::InitializeReference(bad, result), s::Status::InvalidInput);
  EXPECT_EQ(solid18_test::Bytes(result), old);
  ASSERT_EQ(law::InitializeReference(original, result), s::Status::Success);
  EXPECT_EQ(Values(result), accepted_values);
}
TEST(Rear18Reference, AliasedInputAndLateGeometryFailureAreAtomic) {
  law::Reference result;
  ASSERT_EQ(law::InitializeReference(Cube(), result), s::Status::Success);
  const auto values = Values(result);
  ASSERT_EQ(law::InitializeReference(result.input(), result), s::Status::Success);
  EXPECT_EQ(Values(result), values);
  auto& alias = const_cast<s::ReferenceInput&>(result.input());
  alias.source_node_id[7] = alias.source_node_id[0];
  const auto bytes = solid18_test::Bytes(result);
  EXPECT_EQ(law::InitializeReference(alias, result), s::Status::InvalidInput);
  EXPECT_EQ(solid18_test::Bytes(result), bytes);
  auto bad = Cube();
  bad.position_m[7].x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(law::InitializeReference(bad, result), s::Status::InvalidInput);
  EXPECT_EQ(solid18_test::Bytes(result), bytes);
  bad = Cube();
  for (auto& point : bad.position_m) point.z = 0;
  EXPECT_EQ(law::InitializeReference(bad, result), s::Status::InvalidGeometry);
  EXPECT_EQ(solid18_test::Bytes(result), bytes);
  ASSERT_EQ(law::InitializeReference(Cube(), result), s::Status::Success);
  EXPECT_EQ(Values(result), values);
}
}  // namespace rear18_test
