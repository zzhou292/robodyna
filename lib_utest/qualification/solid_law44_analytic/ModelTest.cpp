// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "lib_utest/qualification/extended_solid_model/Fixture.h"
#include "lib_utest/qualification/extended_solid_model/ResidentConfig.h"
#include "lib_src/elements/solids/resident/MaterialUpload.h"

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
} // namespace law44_analytic_test
