// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "OperandBaselineArena.h"

namespace solid_operands_test {
template<class Traits> void CheckRegion(HostRig& rig, const d::FamilyLayout& layout) {
  const auto& region = layout.measurement;
  auto& family = d::FamilyStorage<Traits>(rig.state);
  EXPECT_EQ(region.count, family.count);
  EXPECT_EQ(region.bytes, family.count * sizeof(d::MeasurementOperands<Traits::nodes>));
  EXPECT_GE(region.offset, layout.result_valid.offset + layout.result_valid.bytes);
  EXPECT_LE(region.offset + region.bytes, rig.layout.bytes);
  EXPECT_EQ(region.offset % alignof(d::MeasurementOperands<Traits::nodes>), 0u);
  EXPECT_EQ(family.measurement,
      tl::util::ArenaPointer<d::MeasurementOperands<Traits::nodes>>(rig.arena.data(), region));
  for (std::size_t p = 0; p < family.count; ++p) {
    const auto& value = family.measurement[p];
    EXPECT_EQ(value.work, 0);
    EXPECT_EQ(value.hourglass_work, 0);
    EXPECT_EQ(value.plastic_work, 0);
    EXPECT_EQ(value.native_dt, 0);
    for (unsigned n = 0; n < Traits::nodes; ++n) {
      EXPECT_EQ(value.kick[n], 0);
      EXPECT_EQ(value.drift[n], 0);
    }
  }
}
TEST(SolidOperandBudget, FiveTypedRegionsUploadZeroAndExactHostDeviceCapsRetry) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  CheckRegion<d::Traits18>(rig, rig.layout.solid18);
  CheckRegion<d::Traits24>(rig, rig.layout.solid24);
  CheckRegion<d::Traits6z>(rig, rig.layout.solid6z);
  CheckRegion<d::Traits18Law44>(rig, rig.layout.solid18_law44);
  CheckRegion<d::Traits18Law90>(rig, rig.layout.solid18_law90);
  s::BatchForecast forecast;
  ASSERT_TRUE(s::Batch::Forecast(rig.config, rig.model, forecast));
  EXPECT_EQ(forecast.device_bytes, rig.layout.bytes);
  auto exact = rig.config;
  exact.limits.max_device_bytes = forecast.device_bytes;
  exact.limits.max_host_bytes = forecast.startup_host_bytes;
  s::BatchForecast output;
  ASSERT_TRUE(s::Batch::Forecast(exact, rig.model, output));
  --exact.limits.max_device_bytes;
  output.device_bytes = 53;
  EXPECT_FALSE(s::Batch::Forecast(exact, rig.model, output));
  EXPECT_EQ(output.device_bytes, 53u);
  ++exact.limits.max_device_bytes;
  --exact.limits.max_host_bytes;
  output.startup_host_bytes = 71;
  EXPECT_FALSE(s::Batch::Forecast(exact, rig.model, output));
  EXPECT_EQ(output.startup_host_bytes, 71u);
  ++exact.limits.max_host_bytes;
  ASSERT_TRUE(s::Batch::Forecast(exact, rig.model, output));
  EXPECT_EQ(output.startup_host_bytes, forecast.startup_host_bytes);
}
TEST(SolidOperandBudget, FrozenAlignedArenaIncludesPayloadHeaderAndHostMetadata) {
  namespace prior = d::operand_baseline;
  const auto config = extended_resident_test::Config(376930);
  d::ArenaLayout layout;
  prior::ArenaLayout baseline;
  for (std::size_t n : {1u, 17u, 257u}) {
    ASSERT_TRUE(d::MakeLayout({n,n,n,1,1,18,n,n,1,1}, config, layout));
    ASSERT_TRUE(prior::MakeLayout({n,n,n,1,1,18,n,n,1,1}, config, baseline));
    const auto payload = n * (4 * 160 + 128);
    EXPECT_EQ(layout.bytes - baseline.bytes, payload + sizeof(d::Storage) - sizeof(prior::Storage));
    EXPECT_EQ(layout.staging_bytes, baseline.staging_bytes);
  }
  EXPECT_EQ(sizeof(d::Storage) - sizeof(prior::Storage), 5 * sizeof(void*));
  EXPECT_EQ(sizeof(d::ArenaLayout) - sizeof(prior::ArenaLayout), 5 * sizeof(tl::util::ArenaRegion));
  const d::Counts v5 {908,1991,350,1,8,1024,386,1345,3,1};
  ASSERT_TRUE(d::MakeLayout(v5, config, layout));
  ASSERT_TRUE(prior::MakeLayout({908,1991,350,1,8,1024,386,1345,3,1}, config, baseline));
  EXPECT_EQ(layout.bytes - baseline.bytes, 785600u + sizeof(d::Storage) - sizeof(prior::Storage));
  RecordProperty("count_only_old_device_bytes", baseline.bytes);
  RecordProperty("count_only_new_device_bytes", layout.bytes);
  RecordProperty("exact_device_growth", layout.bytes - baseline.bytes);
  RecordProperty("storage_header_growth", sizeof(d::Storage) - sizeof(prior::Storage));
  RecordProperty("arena_metadata_growth", sizeof(d::ArenaLayout) - sizeof(prior::ArenaLayout));
  auto overflow = v5;
  overflow.solid18_law90 = SIZE_MAX;
  layout.bytes = 83;
  EXPECT_FALSE(d::MakeLayout(overflow, config, layout));
  EXPECT_EQ(layout.bytes, 83u);
  ASSERT_TRUE(d::MakeLayout({1,0,0,1,0,3}, config, layout));
  tl::util::HostArena empty;
  ASSERT_TRUE(empty.Initialize(layout.bytes));
  const auto header = d::RebasedHeader(empty.data(), layout);
  EXPECT_NE(header.solid18.measurement, nullptr);
  EXPECT_EQ(header.solid24.measurement, nullptr);
  EXPECT_EQ(header.solid6z.measurement, nullptr);
  EXPECT_EQ(header.solid18_law44.measurement, nullptr);
  EXPECT_EQ(header.solid18_law90.measurement, nullptr);
}
} // namespace solid_operands_test
