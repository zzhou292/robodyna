// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "ResultValues.h"
#include "lib_src/elements/ShellFormulationOutputRanges.h"

namespace qbat_resident_test {
TEST(QbatResidentHost, ExactArenaBudgetAndOriginalPopulationFootprintStayBounded) {
  batch::Layout layout;
  ASSERT_TRUE(layout.Initialize(4250,4384,0,fe::MaxVehicleShellResidentDeviceBytes));
  EXPECT_LT(layout.bytes,33u*1024*1024);
  const auto exact=layout.bytes;
  const auto previous=Bytes(layout);
  EXPECT_FALSE(layout.Initialize(4250,4384,0,exact-1));
  EXPECT_EQ(Bytes(layout),previous);
  EXPECT_TRUE(layout.Initialize(4250,4384,0,exact));
  EXPECT_FALSE(layout.Initialize(524288,524288,0,fe::MaxVehicleShellResidentDeviceBytes));
  EXPECT_FALSE(layout.Initialize(SIZE_MAX,4,0,SIZE_MAX));
  EXPECT_FALSE(layout.Initialize(1,4,SIZE_MAX,SIZE_MAX));
  EXPECT_FALSE(layout.Initialize(1,4,1,1024*1024));
}
TEST(QbatResidentHost, CompleteCatalogStartupOwnsFourVirginPointsAndOnceOnlyNativeNodeCoefficients) {
  Source source;
  ASSERT_FALSE(HasFailure());
  const auto config=source.Config();
  batch::Layout layout;
  ASSERT_TRUE(layout.Initialize(config.element_count,config.owner.node_count,source.catalog.curve_point_count(),config.max_device_bytes));
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(layout.bytes));
  auto* storage=layout.Construct(arena);
  ASSERT_NE(storage,nullptr);
  qb::BatchDiagnostics diagnostics;
  ASSERT_EQ(batch::BuildStartup(config,source.Scope(),*storage,diagnostics).status,qb::BatchStatus::Success);
  EXPECT_EQ(diagnostics.active_count,1u);
  EXPECT_FALSE(diagnostics.has_completed_interval);
  EXPECT_GT(diagnostics.minimum_native_dt,0);
  for(std::size_t node=0;node<config.owner.node_count;++node) {
    const auto& original=source.binding.nodes()[node];
    EXPECT_DOUBLE_EQ(storage->model.mass[node],original.native.mass);
    EXPECT_DOUBLE_EQ(storage->model.inertia[node],original.native.isotropic_inertia);
  }
  const auto& row=storage->slab[0].element[0];
  EXPECT_TRUE(batch::ValidResult(row,storage->model.element[0].material,0,0));
  for(const auto& point:row.history.point) {
    EXPECT_TRUE(point.surface_active);
    EXPECT_EQ(point.material.plastic_strain,0);
    EXPECT_EQ(point.material.filtered_rate_per_s,0);
  }
  EXPECT_EQ(storage->model.element[0].material.curve.count,0u);
  EXPECT_EQ(storage->model.element[0].material.curve.plastic_strain,nullptr);
  // The source catalog also owns table rows for the other two physical layers.
  EXPECT_EQ(storage->model.curve_points,3u);
  EXPECT_DOUBLE_EQ(storage->model.curve_x[2],source.fixture.x[2]);
}
TEST(QbatResidentHost, LatePointShapeFailureAndIntervalRetryPreserveAcceptedValues) {
  qbat_force_test::Fixture fixture;
  auto element=Element(fixture);
  const auto untouched_element=Bytes(element);
  qb::BatchResult accepted;
  ASSERT_EQ(batch::InitializeResult(element,accepted),qb::Status::kSuccess);
  auto interval=qbat_force_test::Path(fixture,0);
  qb::BatchResult trial;
  ASSERT_EQ(batch::Advance(element,accepted,interval,trial),qb::Status::kSuccess);
  ASSERT_TRUE(batch::ValidResult(trial,element.material,interval.dt,1));
  const auto prior=Bytes(trial);
  interval.omega_midpoint[3].z=std::numeric_limits<double>::infinity();
  EXPECT_NE(batch::Advance(element,accepted,interval,trial),qb::Status::kSuccess);
  EXPECT_EQ(Bytes(trial),prior);
  interval=qbat_force_test::Path(fixture,0);
  qb::BatchResult retry;
  ASSERT_EQ(batch::Advance(element,accepted,interval,retry),qb::Status::kSuccess);
  EXPECT_EQ(ResultValues(retry),ResultValues(trial));
  EXPECT_EQ(Bytes(element),untouched_element);
  auto invalid=trial;
  invalid.point[3].material.equivalent_stress_pa=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(batch::ValidResult(invalid,element.material,interval.dt,1));
  invalid=trial;
  const unsigned char invalid_flag=2;
  std::memcpy(&invalid.history.point[3].surface_active,&invalid_flag,1);
  EXPECT_FALSE(batch::ValidResult(invalid,element.material,interval.dt,1));
  invalid=trial;
  invalid.stamp.sample_index=2;
  EXPECT_FALSE(batch::ValidResult(invalid,element.material,interval.dt,1));
}
TEST(QbatResidentHost, TablePoolOffsetsRebaseToOwnedBackingWithoutCurveDuplication) {
  qbat_catalog_test::Fixture fixture;
  fixture.materials[2].hardening=tl::material::ShellPlasticityHardeningKind::Tabulated;
  fixture.materials[2].curve_id=101;
  fixture.materials[2].linear={};
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(fixture.Geometry()).status,fe::ShellBindingStatus::Success);
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeFormulations(binding,fixture.Input()).status,fe::ShellPlasticityBindingStatus::Success);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,fixture.failures.data(),fixture.failures.size()).status,
      fe::ShellPlasticityBindingStatus::Success);
  Source configuration;
  const auto config=configuration.Config();
  batch::Layout layout;
  ASSERT_TRUE(layout.Initialize(1,binding.node_count(),catalog.curve_point_count(),1024*1024));
  tl::util::HostArena first,second;
  ASSERT_TRUE(first.Initialize(layout.bytes));
  ASSERT_TRUE(second.Initialize(layout.bytes));
  auto* host=layout.Construct(first);
  ASSERT_NE(host,nullptr);
  qb::BatchDiagnostics diagnostics;
  ASSERT_EQ(batch::BuildStartup(config,{&binding,&catalog,&failure},*host,diagnostics).status,qb::BatchStatus::Success);
  const auto& parameters=host->model.element[0].material;
  EXPECT_EQ(parameters.curve.count,3u);
  EXPECT_EQ(parameters.curve.plastic_strain,host->model.curve_x);
  EXPECT_EQ(parameters.curve.yield_stress_pa,host->model.curve_y);
  const auto rebased=layout.Rebase(*host,second.data());
  ASSERT_TRUE(batch::RebaseMaterials(*host,rebased,catalog));
  EXPECT_EQ(host->model.element[0].material.curve.plastic_strain,rebased.model.curve_x);
  EXPECT_NE(rebased.model.curve_x,host->model.curve_x);
  EXPECT_EQ(host->model.curve_points,3u);
}
TEST(QbatResidentHost, EqualScopesWithIndependentBackingHaveDistinctOutputRanges) {
  Source first,second;
  ASSERT_TRUE(first.catalog.SameScope(second.catalog));
  ASSERT_TRUE(first.failure.SameScope(second.failure));
  ASSERT_NE(first.binding.nodes().data(),second.binding.nodes().data());
  const auto* alias=second.binding.nodes().data();
  const auto bytes=Bytes(*alias);
  EXPECT_TRUE(fe::shell_formulation_detail::OutputDisjoint(first.Scope(),alias,sizeof(*alias)));
  EXPECT_FALSE(fe::shell_formulation_detail::OutputDisjoint(second.Scope(),alias,sizeof(*alias)));
  EXPECT_EQ(Bytes(*alias),bytes);
}
} // namespace qbat_resident_test
