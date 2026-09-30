// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "lib_utest/qualification/extended_solid_model/Fixture.h"
#include "lib_utest/qualification/extended_solid_model/ResidentConfig.h"
#include "lib_src/elements/solids/resident/MaterialUpload.h"
#include "lib_src/elements/solids/resident/ResultChecks.h"

namespace law44_analytic_test {
namespace solids = tl::fea::solids;
namespace resident = solids::batch_detail;
namespace {
void SetAnalytic(extended_model_test::Fixture& f) {
  auto source = f.input44[0].reference.input();
  source.density_kg_m3 = Airbag().material.density_kg_m3;
  ASSERT_EQ(tl::fea::solid18::law44::InitializeReference(source, f.input44[0].reference),
      tl::fea::solid18::Status::Success);
  f.input44[0].material = Airbag();
}
}
TEST(SolidLaw44AnalyticModel, MixedPoolOwnsAnalyticIdentityAndLastMaterialFailureRetries) {
  extended_model_test::Fixture f; SetAnalytic(f); f.Repeat44();
  const auto domain = f.Domain();
  solids::Model model;
  auto changed = f.input44[1].material.material.analytic;
  changed.b_pa = std::nextafter(changed.b_pa, INFINITY);
  ASSERT_EQ(law::PrepareAnalytic(f.input44[1].material.material, changed, f.input44[1].material), law::Status::Ok);
  EXPECT_EQ(model.Initialize(domain, f.Input()).status, solids::ModelStatus::MaterialMismatch);
  EXPECT_FALSE(model.prepared());
  f.input44[1].material = f.input44[0].material;
  f.foam_y[2] = NAN;
  EXPECT_EQ(model.Initialize(domain, f.Input()).status, solids::ModelStatus::InvalidInput);
  EXPECT_FALSE(model.prepared());
  f.foam_y[2] = 50e6;
  ASSERT_TRUE(model.Initialize(domain, f.Input()));
  ASSERT_EQ(model.materials44().size(), 1u);
  Empty(model.materials44()[0].value);
  EXPECT_EQ(model.solid18_law44()[0].material_index, model.solid18_law44()[1].material_index);
  const auto owned_b = model.materials44()[0].value.material.analytic.b_pa;
  f.input44.clear();
  const auto retained = model;
  EXPECT_TRUE(Bits(retained.materials44()[0].value.material.analytic.b_pa, owned_b));
  EXPECT_EQ(retained.solid18_law44()[0].domain_nodes[4], retained.solid18_law44()[0].domain_nodes[5]);
}
TEST(SolidLaw44AnalyticModel, EmptyCurveArenaExactCapsRelocationAndNoncanonicalRetry) {
  extended_model_test::Fixture f; SetAnalytic(f);
  const auto domain = f.Domain();
  auto input = f.Input();
  input.solid18 = {nullptr, 0}; input.solid24 = {nullptr, 0};
  input.solid6z = {nullptr, 0}; input.solid18_law90 = {nullptr, 0};
  solids::Model model;
  f.input44[0].material.curve.plastic_strain = reinterpret_cast<const double*>(1);
  EXPECT_EQ(model.Initialize(domain, input).status, solids::ModelStatus::InvalidInput);
  f.input44[0].material.curve = {};
  ASSERT_TRUE(model.Initialize(domain, input));
  solids::ModelLimits limits; limits.max_host_bytes = model.startup_payload_bytes() - 1;
  solids::Model retry;
  EXPECT_EQ(retry.Initialize(domain, input, limits).status, solids::ModelStatus::ResourceLimit);
  limits.max_host_bytes += 1;
  ASSERT_TRUE(retry.Initialize(domain, input, limits));
  auto config = extended_model_test::ResidentConfig(domain.node_count());
  config.profile = solids::BatchProfile::PhysicalCinExtendedLaw44Law90V2;
  resident::ArenaLayout layout;
  ASSERT_TRUE(resident::Plan(config, model, layout));
  EXPECT_EQ(layout.curves.count, 0u); EXPECT_EQ(layout.curves.bytes, 0u);
  const auto relocated = resident::ExpectedMaterial44(model, 0, reinterpret_cast<double*>(4096));
  Empty(relocated);
  EXPECT_TRUE(tl::fea::solid18::law44::detail::SameMaterial(relocated, model.materials44()[0].value));
  auto bad = relocated; bad.material.hardening = law::HardeningKind::Tabulated;
  EXPECT_FALSE(tl::fea::solid18::law44::detail::SameMaterial(bad, relocated));
  config.limits.max_device_bytes = layout.bytes - 1;
  resident::ArenaLayout unchanged; unchanged.bytes = 29;
  EXPECT_EQ(resident::Plan(config, model, unchanged).status, solids::BatchStatus::ResourceLimit);
  EXPECT_EQ(unchanged.bytes, 29u);
  config.limits.max_device_bytes += 1;
  ASSERT_TRUE(resident::Plan(config, model, unchanged));
  EXPECT_EQ(unchanged.bytes, layout.bytes);
}
TEST(SolidLaw44AnalyticModel, MissingAndForeignReceiptsCannotBeRepairedByModelCopy) {
  const auto table = law44_solid_test::Parameters();
  auto forged_table = table;
  forged_table.analytic_preparation = Airbag().analytic_preparation;
  EXPECT_FALSE(law::detail::ParametersValid(forged_table));
  EXPECT_FALSE(tl::fea::solid18::law44::detail::SameMaterial(table, forged_table));
  for (bool foreign : {false, true}) {
    extended_model_test::Fixture f; SetAnalytic(f);
    const auto admitted = f.input44[0].material;
    const auto domain = f.Domain();
    f.input44[0].material.analytic_preparation = foreign ? Capped(.6).analytic_preparation : law::AnalyticPreparation{};
    solids::Model model;
    EXPECT_EQ(model.Initialize(domain, f.Input()).status, solids::ModelStatus::InvalidInput);
    EXPECT_FALSE(model.prepared());
    f.input44[0].material = admitted;
    ASSERT_TRUE(model.Initialize(domain, f.Input()));
    EXPECT_TRUE(law::detail::PreparedHardeningValid(model.materials44()[0].value));
  }
}
TEST(SolidLaw44AnalyticModel, HistoryAndCacheIdentityRejectLostReceiptBeforeRetry) {
  extended_model_test::Fixture f; SetAnalytic(f);
  const auto domain = f.Domain();
  solids::Model model;
  ASSERT_TRUE(model.Initialize(domain, f.Input()));
  const auto& parent = model.solid18_law44()[0];
  const auto& material = model.materials44()[0].value;
  namespace rear = tl::fea::solid18::law44;
  rear::ForceTrial initial;
  ASSERT_EQ(rear::InitializeForce(parent.reference, material, {}, initial), tl::fea::solid18::Status::Success);
  resident::State<resident::Traits18Law44> accepted;
  accepted.history = initial.proposed_history;
  ASSERT_TRUE(resident::Traits18Law44::Capture(initial, accepted.cache));
  ASSERT_TRUE(resident::ValidResult(parent, material, accepted, 0, 0));
  resident::Traits18Law44::Interval interval;
  interval.dt_s = 1e-6;
  interval.sample_index = 1;
  for (unsigned n = 0; n < 8; ++n) interval.position_endpoint_m[n] = parent.reference.input().position_m[n];
  for (bool foreign : {false, true}) {
    auto corrupt = accepted;
    auto& history_material = const_cast<law::Parameters&>(corrupt.history.material());
    history_material.analytic_preparation = foreign ? Capped(.6).analytic_preparation : law::AnalyticPreparation{};
    EXPECT_FALSE(tl::fea::solid18::law44::detail::SameMaterial(material, history_material));
    EXPECT_FALSE(resident::ValidResult(parent, material, corrupt, 0, 0));
    auto unchanged = initial;
    unchanged.rhs_force_n[0].x = 917;
    EXPECT_EQ(rear::EvaluateForce(parent.reference, corrupt.history, interval, material, unchanged),
              tl::fea::solid18::Status::InvalidInput);
    EXPECT_EQ(unchanged.rhs_force_n[0].x, 917);
    EXPECT_EQ(unchanged.proposed_history.stamp().sample_index, 0u);
  }
  rear::ForceTrial next;
  ASSERT_EQ(rear::EvaluateForce(parent.reference, accepted.history, interval, material, next),
            tl::fea::solid18::Status::Success);
  resident::State<resident::Traits18Law44> retry;
  retry.history = next.proposed_history;
  ASSERT_TRUE(resident::Traits18Law44::Capture(next, retry.cache));
  EXPECT_TRUE(resident::ValidResult(parent, material, retry, interval.dt_s, 1));
}
} // namespace law44_analytic_test
