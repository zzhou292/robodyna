// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "FrozenArena.h"

namespace solid_validation_test {
TEST(SolidCandidateBudget, AllFiveByteRegionsAreOwnedAlignedAndFullyCharged) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  const d::FamilyLayout* regions[] {&rig.layout.solid18, &rig.layout.solid24,
      &rig.layout.solid6z, &rig.layout.solid18_law44, &rig.layout.solid18_law90};
  const std::uint8_t* flags[] {rig.state.solid18.result_valid, rig.state.solid24.result_valid,
      rig.state.solid6z.result_valid, rig.state.solid18_law44.result_valid,
      rig.state.solid18_law90.result_valid};
  for (unsigned f = 0; f < 5; ++f) {
    const auto& r = regions[f]->result_valid;
    EXPECT_EQ(r.count, 1u);
    EXPECT_EQ(r.bytes, r.count);
    EXPECT_GE(r.offset, regions[f]->status.offset + regions[f]->status.bytes);
    EXPECT_LE(r.offset + r.bytes, rig.layout.bytes);
    EXPECT_EQ(flags[f], tl::util::ArenaPointer<std::uint8_t>(rig.arena.data(), r));
    EXPECT_EQ(*flags[f], 255);
  }
  EXPECT_EQ(rig.layout.header.bytes, sizeof(d::Storage));
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
}
TEST(SolidCandidateBudget, EmptyOverflowAndCountOnlyForecastUseExactOldAndNewLayouts) {
  namespace prior = d::baseline_layout;
  const auto config = extended_resident_test::Config(376930);
  for (std::size_t n : {1u, 17u, 257u}) {
    const d::Counts counts {n, n, n, 1, 1, 18, n, n, 1, 1};
    const prior::Counts old_counts {n, n, n, 1, 1, 18, n, n, 1, 1};
    d::ArenaLayout layout;
    prior::ArenaLayout baseline;
    ASSERT_TRUE(d::MakeLayout(counts, config, layout));
    ASSERT_TRUE(prior::MakeLayout(old_counts, config, baseline));
    EXPECT_EQ(layout.staging_bytes, baseline.staging_bytes);
    // Byte flags can occupy padding that already followed an odd int count.
    // Compare actual aligned layouts, not an unaligned N-byte growth estimate.
    EXPECT_GE(layout.bytes, baseline.bytes + sizeof(d::Storage) - sizeof(prior::Storage));
  }
  d::ArenaLayout layout;
  ASSERT_TRUE(d::MakeLayout({1, 0, 0, 1, 0, 3}, config, layout));
  tl::util::HostArena empty;
  ASSERT_TRUE(empty.Initialize(layout.bytes));
  const auto header = d::RebasedHeader(empty.data(), layout);
  EXPECT_EQ(header.solid24.result_valid, nullptr);
  EXPECT_EQ(header.solid6z.result_valid, nullptr);
  EXPECT_EQ(header.solid18_law44.result_valid, nullptr);
  EXPECT_EQ(header.solid18_law90.result_valid, nullptr);
  const d::Counts v5 {908, 1991, 350, 1, 8, 1024, 386, 1345, 3, 1};
  ASSERT_TRUE(d::MakeLayout(v5, config, layout));
  prior::ArenaLayout baseline;
  ASSERT_TRUE(prior::MakeLayout({908, 1991, 350, 1, 8, 1024, 386, 1345, 3, 1}, config, baseline));
  RecordProperty("count_only_old_device_bytes", baseline.bytes);
  RecordProperty("count_only_new_device_bytes", layout.bytes);
  RecordProperty("exact_device_growth", layout.bytes - baseline.bytes);
  RecordProperty("storage_header_growth", sizeof(d::Storage) - sizeof(prior::Storage));
  auto overflow = v5;
  overflow.solid18_law90 = SIZE_MAX;
  layout.bytes = 83;
  EXPECT_FALSE(d::MakeLayout(overflow, config, layout));
  EXPECT_EQ(layout.bytes, 83u);
}
} // namespace solid_validation_test
