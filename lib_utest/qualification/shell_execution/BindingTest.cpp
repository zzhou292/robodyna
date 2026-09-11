// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace shell_execution_test {
TEST(ShellExecutionBinding, OriginalPartAndMergedChildRetainCompleteRowsAndOneLedger) {
  Fixture f;
  fe::ShellExecutionBinding value;
  ASSERT_EQ(value.Initialize(f.catalog,f.ledger,f.rigid).status,Status::Success);
  EXPECT_EQ(value.counts().rigid_skin,3u);
  EXPECT_EQ(value.counts().constitutive,4u);
  EXPECT_EQ(value.counts().material_points,11u);
  const auto* primary = value.parent(Family::Qeph,0);
  const auto* child = value.parent(Family::T3,3);
  ASSERT_NE(primary,nullptr);
  ASSERT_NE(child,nullptr);
  EXPECT_EQ(primary->part_index,0u);
  EXPECT_EQ(child->part_index,1u);
  EXPECT_EQ(primary->root_index,child->root_index);
  EXPECT_EQ(child->source.source_part_id,2000u);
  // Q1 and QBAT share all physical nodes with the rigid skin; no role inference.
  EXPECT_EQ(value.parent(Family::Qeph,1)->law,Law::LayeredLaw44Nip3);
  EXPECT_EQ(value.parent(Family::Qbat,0)->material_points,4u);
  EXPECT_EQ(value.parent(Family::Qeph,1)->part_index,SIZE_MAX);
  ASSERT_TRUE(value.coefficients()->Matches(f.ledger));
  qbat_binding_test::Exact(value.shells()->totals(),f.shells.totals());
  const auto nodes = f.ledger.nodes();
  const auto retained = value.coefficients()->nodes();
  EXPECT_EQ(nodes.data(),retained.data());
  auto copy = value;
  EXPECT_TRUE(copy.Matches(value));
  EXPECT_TRUE(copy.Matches(f.catalog,f.ledger));
  EXPECT_EQ(value.Initialize(f.catalog,f.ledger,f.rigid).status,Status::AlreadyInitialized);
  EXPECT_EQ(copy.parent(SIZE_MAX),nullptr);
  EXPECT_EQ(copy.parent(Family::None,0),nullptr);
  EXPECT_EQ(copy.parent(Family::T3,4),nullptr);
}
TEST(ShellExecutionBinding, LateWrongOriginalPidIsRejectedDespiteEqualMergedRoot) {
  Fixture f;
  Source wrong;
  wrong.parents.back().source_part_id = 1000;
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeExecutionCatalog(f.shells,wrong.Catalog()).status,Status::Success);
  fe::ShellExecutionBinding value;
  const auto bad = value.Initialize(catalog,f.ledger,f.rigid);
  EXPECT_EQ(bad.status,Status::IdentityMismatch);
  EXPECT_EQ(bad.entry,6u);
  EXPECT_FALSE(value.prepared());
  EXPECT_EQ(value.catalog(),nullptr);
  ASSERT_EQ(value.Initialize(f.catalog,f.ledger,f.rigid).status,Status::Success);
}
TEST(ShellExecutionBinding, MissingPartDisguisedRoleAndForeignLedgerReject) {
  Fixture f;
  Source missing;
  missing.parents.back().source_part_id = 777;
  fe::ShellBatchPlasticityBinding missing_catalog;
  ASSERT_EQ(missing_catalog.InitializeExecutionCatalog(f.shells,missing.Catalog()).status,Status::Success);
  fe::ShellExecutionBinding value;
  EXPECT_EQ(value.Initialize(missing_catalog,f.ledger,f.rigid).status,Status::IdentityMismatch);
  Source disguised;
  disguised.base.materials[0].law = Law::LayeredLaw1Nip3;
  disguised.base.sections[0].through_thickness_points = 3;
  disguised.base.sections[0].formulation = fe::ShellSectionFormulation::LayeredNip3;
  fe::ShellBatchPlasticityBinding hidden;
  ASSERT_EQ(hidden.InitializeExecutionCatalog(f.shells,disguised.Catalog()).status,Status::Success);
  EXPECT_EQ(value.Initialize(hidden,f.ledger,f.rigid).status,Status::IdentityMismatch);
  const auto& node = f.domain.nodes()[0];
  const fe::ElementMassSource mass{800,node.source_id,0,2};
  fe::ElementMassContributions extra;
  ASSERT_TRUE(extra.Initialize(f.domain,{77,1,&mass,1}));
  fe::NodalCoefficientLedger other;
  ASSERT_TRUE(other.InitializeWithElementMass({{&f.mapping,nullptr,nullptr},&extra}));
  EXPECT_EQ(value.Initialize(f.catalog,other,f.rigid).status,Status::IdentityMismatch);
  EXPECT_FALSE(value.prepared());
  EXPECT_EQ(value.Initialize(f.catalog,f.ledger,f.rigid).status,Status::Success);
}
TEST(ShellExecutionBinding, ExactStartupCapsAndFailedForecastPreserveOutputThenRetry) {
  Fixture f;
  fe::ShellExecutionForecast forecast;
  ASSERT_EQ(fe::ShellExecutionBinding::Preflight(f.catalog,f.ledger,f.rigid,{},&forecast).status,Status::Success);
  EXPECT_GT(forecast.startup_payload_bytes,forecast.owned_payload_bytes);
  auto limits = fe::ShellExecutionLimits{};
  limits.max_host_bytes = forecast.startup_payload_bytes-1;
  fe::ShellExecutionForecast sentinel{91,92};
  EXPECT_EQ(fe::ShellExecutionBinding::Preflight(f.catalog,f.ledger,f.rigid,limits,&sentinel).status,Status::ResourceLimit);
  EXPECT_EQ(sentinel.owned_payload_bytes,91u);
  EXPECT_EQ(sentinel.startup_payload_bytes,92u);
  fe::ShellExecutionBinding value;
  EXPECT_EQ(value.Initialize(f.catalog,f.ledger,f.rigid,limits).status,Status::ResourceLimit);
  EXPECT_FALSE(value.prepared());
  limits.max_host_bytes = forecast.startup_payload_bytes;
  ASSERT_EQ(value.Initialize(f.catalog,f.ledger,f.rigid,limits).status,Status::Success);
  EXPECT_EQ(value.forecast().startup_payload_bytes,forecast.startup_payload_bytes);
}
TEST(ShellExecutionBinding, RetainedCatalogPartAndDomainSurviveAllInputHandles) {
  auto retained = [] {
    Fixture f;
    fe::ShellExecutionBinding value;
    EXPECT_EQ(value.Initialize(f.catalog,f.ledger,f.rigid).status,Status::Success);
    return value;
  }();
  ASSERT_TRUE(retained.prepared());
  ASSERT_NE(retained.parent(Family::T3,3),nullptr);
  EXPECT_EQ(retained.parent(Family::T3,3)->source.source_part_id,2000u);
  EXPECT_EQ(retained.rigid()->parts()->topology()->merge_count(),1u);
  EXPECT_EQ(retained.domain()->source_instance_id(),77u);
  fe::sections::PointParameters material;
  ASSERT_TRUE(retained.catalog()->Parameters(Family::Qeph,1,&material));
  ASSERT_EQ(material.curve.count,3u);
  EXPECT_EQ(material.curve.yield_stress_pa[2],20e6);
  qbat_binding_test::Reduction(*retained.shells());
}
} // namespace shell_execution_test
