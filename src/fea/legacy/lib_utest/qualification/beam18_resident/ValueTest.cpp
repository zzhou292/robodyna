// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "lib_src/elements/beam18/resident/Storage.h"
#include "lib_src/elements/beam18/resident/ResultChecks.h"
namespace beam18_resident_test {
namespace d = b::batch_detail;
TEST(BeamResidentValues, VehicleSizedArenaIsBoundedWithoutAllocatingASecondDomain) {
  b::BatchConfig config;
  config.owner.node_count = 372435;
  config.cin_attachment_count = 11165;
  config.cin_witness_count = 13173;
  d::ArenaLayout layout;
  ASSERT_TRUE(d::MakeLayout({142, 4, 4*46}, config, layout));
  EXPECT_EQ(layout.parents.count, 142u);
  EXPECT_EQ(layout.slab[0].count, 142u);
  EXPECT_EQ(layout.slab[1].count, 142u);
  EXPECT_EQ(layout.curves.count, 2u*4*46);
  EXPECT_EQ(layout.status.count, 142u);
  EXPECT_LT(layout.bytes, 8u << 20);
  RecordProperty("original_count_device_arena_bytes", std::to_string(layout.bytes));
  RecordProperty("original_domain_initial_proof_bytes", std::to_string(layout.proof.bytes));
  const auto bytes = layout.bytes;
  config.limits.max_device_bytes = bytes-1;
  EXPECT_FALSE(d::MakeLayout({142, 4, 4*46}, config, layout));
  EXPECT_EQ(layout.bytes, bytes);
}
TEST(BeamResidentValues, CompleteModelOwnerFixtureContainsOrdinaryPartAndPlainEndpoints) {
  OwnerFixture fixture;
  ASSERT_TRUE(fixture.model.prepared());
  EXPECT_EQ(fixture.model.parents().size(), 3u);
  EXPECT_EQ(fixture.model.materials().size(), 1u);
  EXPECT_EQ(fixture.ledger.scope().uncovered_nodes, 0u);
  const auto parents = fixture.model.parents();
  EXPECT_TRUE(fixture.binding.FindMember(parents[0].domain_nodes[0]));
  EXPECT_TRUE(fixture.binding.FindMember(parents[1].domain_nodes[0]));
  EXPECT_FALSE(fixture.binding.FindMember(parents[2].domain_nodes[0]));
  EXPECT_FALSE(fixture.binding.FindMember(parents[2].domain_nodes[1]));
}
TEST(BeamResidentValues, OneArenaAndExactCompleteHostCapRejectBeforeAllocation) {
  OwnerFixture fixture;
  auto c = fixture.Configuration();
  b::BatchForecast forecast;
  ASSERT_TRUE(b::Batch::Forecast(c, fixture.model, forecast));
  EXPECT_GT(forecast.startup_host_bytes, fixture.model.owned_payload_bytes() + forecast.device_bytes);
  c.limits.max_device_bytes = forecast.device_bytes;
  c.limits.max_host_bytes = forecast.startup_host_bytes;
  b::BatchForecast exact;
  ASSERT_TRUE(b::Batch::Forecast(c, fixture.model, exact));
  EXPECT_EQ(exact.device_bytes, forecast.device_bytes);
  --c.limits.max_host_bytes;
  EXPECT_EQ(b::Batch::Forecast(c, fixture.model, exact).status, b::BatchStatus::ResourceLimit);
  EXPECT_EQ(exact.startup_host_bytes, forecast.startup_host_bytes);
  c = fixture.Configuration(); c.limits.max_device_bytes = forecast.device_bytes - 1;
  EXPECT_EQ(b::Batch::Forecast(c, fixture.model, exact).status, b::BatchStatus::ResourceLimit);
  c = fixture.Configuration(); c.owner.epoch = 1;
  EXPECT_EQ(b::Batch::Forecast(c, fixture.model, exact).status, b::BatchStatus::InvalidInput);
}
TEST(BeamResidentValues, UploadedCurvesAndParentsHaveOwnedLifetimeAndDeviceIdentity) {
  OwnerFixture fixture;
  d::ArenaLayout layout;
  const auto config = fixture.Configuration();
  ASSERT_TRUE(d::Plan(config, fixture.model, layout));
  tl::util::HostArena upload;
  ASSERT_TRUE(upload.Initialize(layout.bytes));
  d::Storage header;
  ASSERT_TRUE(d::BuildUpload(config, fixture.model, upload, layout, header));
  const auto& source = fixture.model.materials()[0].value;
  EXPECT_NE(source.curve.plastic_strain, header.materials[0].curve.plastic_strain);
  for (unsigned p = 0; p < source.curve.count; ++p)
    EXPECT_EQ(source.curve.yield_stress_pa[p], header.materials[0].curve.yield_stress_pa[p]);
  for (unsigned p = 0; p < 3; ++p)
    EXPECT_EQ(header.parents[p].domain_nodes[1], fixture.model.parents()[p].domain_nodes[1]);
  b::ForceTrial value;
  const auto& parent = header.parents[0];
  ASSERT_EQ(b::InitializeForce(parent.reference, header.materials[0], {}, value), b::Status::Success);
  ASSERT_TRUE(d::ValidResult(parent, header.materials[0], value, 0, 0));
  auto* fake_device = reinterpret_cast<void*>(std::uintptr_t(0x10000000));
  d::RebaseCurves(fixture.model, layout, fake_device, header);
  const auto expected = d::ExpectedMaterial(fixture.model, 0, tl::util::ArenaPointer<double>(fake_device, layout.curves));
  EXPECT_EQ(expected.curve.plastic_strain, header.materials[0].curve.plastic_strain);
  b::ForceHistoryWriter::Store(parent.reference, expected, value.proposed_history.values(), {}, value.proposed_history);
  // Inaccessible address is compared only; no host curve dereference occurs.
  EXPECT_TRUE(d::ValidResult(parent, expected, value, 0, 0));
  auto wrong = expected; ++wrong.curve.count;
  EXPECT_FALSE(d::ValidResult(parent, wrong, value, 0, 0));
}
TEST(BeamResidentValues, CompleteCacheValidationAndPointerFreeReadbackDetectLateDifferences) {
  OwnerFixture fixture;
  ASSERT_TRUE(fixture.model.prepared());
  const auto& parent = fixture.model.parents()[2];
  const auto material = fixture.model.materials()[parent.material_index].value;
  b::ForceTrial value;
  ASSERT_EQ(b::InitializeForce(parent.reference, material, {}, value), b::Status::Success);
  const auto output = d::Read(value);
  EXPECT_EQ(output.stamp.sample_index, 0u);
  EXPECT_EQ(output.history.point[3].curve_cursor, 0u);
  value.rhs_couple_nm[1].z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(d::ValidResult(parent, material, value, 0, 0));
  ASSERT_EQ(b::InitializeForce(parent.reference, material, {}, value), b::Status::Success);
  value.point[3].history.stress_pa[2] = 1.;
  EXPECT_FALSE(d::ValidResult(parent, material, value, 0, 0));
  ASSERT_EQ(b::InitializeForce(parent.reference, material, {}, value), b::Status::Success);
  EXPECT_TRUE(d::ValidResult(parent, material, value, 0, 0));
}
} // namespace beam18_resident_test
