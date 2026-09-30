// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../physical_publication/Fixture.h"
#include "lib_src/elements/publication/physical_activity/Storage.h"
namespace physical_activity_test {
namespace fe = tl::fea; namespace a = fe::physical_activity;
TEST(PhysicalActivityHost, ExactForecastBoundsAndOverlappingSourceAreAtomic) {
  physical_publication_test::Fixture fixture;
  fe::PhysicalActivityForecast f;
  ASSERT_EQ(fe::PhysicalActivitySnapshot::Preflight(fixture.physical, {}, f).status, fe::PhysicalActivityStatus::Ok);
  RecordProperty("owned_host_bytes", std::to_string(f.owned_host_bytes));
  RecordProperty("startup_host_bytes", std::to_string(f.startup_host_bytes));
  RecordProperty("preparation_host_bytes", std::to_string(f.preparation_host_bytes));
  RecordProperty("device_bytes", std::to_string(f.device_bytes));
  EXPECT_EQ(f.qeph_count, 2u); EXPECT_EQ(f.t3_count, 1u); EXPECT_EQ(f.qbat_count, 1u);
  EXPECT_EQ(f.preparation_device_bytes, 0u);
  auto held = f; fe::PhysicalActivityLimits limits;
  limits.max_device_bytes = f.device_bytes - 1;
  EXPECT_EQ(fe::PhysicalActivitySnapshot::Preflight(fixture.physical, limits, held).status, fe::PhysicalActivityStatus::ResourceLimit);
  EXPECT_EQ(held.device_bytes, f.device_bytes);
  limits = {}; limits.max_host_bytes = f.owned_host_bytes + f.preparation_host_bytes - 1;
  EXPECT_EQ(fe::PhysicalActivitySnapshot::Preflight(fixture.physical, limits, held).status, fe::PhysicalActivityStatus::ResourceLimit);
  limits = {}; limits.max_startup_host_bytes = f.startup_host_bytes - 1;
  EXPECT_EQ(fe::PhysicalActivitySnapshot::Preflight(fixture.physical, limits, held).status, fe::PhysicalActivityStatus::ResourceLimit);
  auto* alias = reinterpret_cast<fe::PhysicalActivityForecast*>(const_cast<fe::NodalCoefficientNode*>(fixture.ledger.nodes().data()));
  EXPECT_EQ(fe::PhysicalActivitySnapshot::Preflight(fixture.physical, {}, *alias).status, fe::PhysicalActivityStatus::InvalidInput);
}
TEST(PhysicalActivityHost, LayoutOverflowAndZeroFamilyRemainBounded) {
  a::Layout layout;
  EXPECT_FALSE(a::MakeLayout(SIZE_MAX, SIZE_MAX, layout)); EXPECT_EQ(layout.bytes, 0u);
  ASSERT_TRUE(a::MakeLayout(0, 1024, layout)); EXPECT_EQ(layout.bytes, 2*sizeof(a::FamilyControl));
  a::Layout exact; ASSERT_TRUE(a::MakeLayout(17, 1024, exact));
  EXPECT_FALSE(a::MakeLayout(17, exact.bytes - 1, layout));
}
TEST(PhysicalActivityHost, TypedErrorDecodePreservesPhaseAndIndex) {
  a::FamilyControl c;
  EXPECT_EQ(a::Decode(c, fe::PhysicalActivityFamily::T3).status, fe::PhysicalActivityStatus::Ok);
  c.first_error = (std::uint64_t(fe::PhysicalActivityStage::Transition) << 56) | (42ull << 16) | 1;
  const auto r = a::Decode(c, fe::PhysicalActivityFamily::T3);
  EXPECT_EQ(r.status, fe::PhysicalActivityStatus::Reactivation); EXPECT_EQ(r.family_index, 42u);
  EXPECT_EQ(r.family, fe::PhysicalActivityFamily::T3); EXPECT_EQ(r.stage, fe::PhysicalActivityStage::Transition);
}
TEST(PhysicalActivityHost, OtherFamilyRemovalObligationsStayClosed) {
  physical_publication_test::Fixture f; fe::ShellPhysicalDiagnostics d;
  d.has_qbat = true; d.qbat.element_count = 1; d.qbat.active_count = 0;
  EXPECT_EQ(a::GuardOtherFamilies(f.physical, d).status, fe::PhysicalActivityStatus::UnsupportedRemoval);
  d.qbat.active_count = 1; d.has_type25 = true; d.type25.element_count = 2;
  EXPECT_EQ(a::GuardOtherFamilies(f.physical, d).family, fe::PhysicalActivityFamily::Type25);
  d.type25.active_count = 2; d.has_solids = true;
  EXPECT_EQ(a::GuardOtherFamilies(f.physical, d).status, fe::PhysicalActivityStatus::SourceMismatch);
}
}
