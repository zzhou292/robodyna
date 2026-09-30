// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/runtime/Layout.h"
#include "../radioss_type25_lifecycle/Fixture.h"
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <numeric>
namespace native_diagnostic_test {
namespace n=tlfea::contact::radioss_type25;
namespace rd=n::runtime_detail;
inline std::uint64_t Bits(double value) {
  std::uint64_t out;std::memcpy(&out,&value,sizeof(out));return out;
}
inline void Same(const rd::Control& actual,const rd::Control& expected) {
  EXPECT_EQ(actual.failure,expected.failure);
  EXPECT_EQ(actual.required_sliding,expected.required_sliding);
  EXPECT_EQ(actual.required_candidates,expected.required_candidates);
  EXPECT_EQ(actual.kept,expected.kept);EXPECT_EQ(actual.active,expected.active);
  EXPECT_EQ(Bits(actual.elastic_energy),Bits(expected.elastic_energy));
  EXPECT_EQ(Bits(actual.damping_work),Bits(expected.damping_work));
  EXPECT_EQ(Bits(actual.friction_work),Bits(expected.friction_work));
}
inline rd::Control Seed() {
  rd::Control seed;seed.required_sliding=713;seed.required_candidates=997;
  seed.kept=331;seed.active=17;seed.elastic_energy=2.5;
  seed.damping_work=-3.125;seed.friction_work=.75;return seed;
}
// Existing native lifecycle source fixture supplies the admitted real arena
// shape. These tests change only response operands, never mechanics sources.
struct LayoutFixture {
  type25_lifecycle_test::Fixture fixture;
  n::FixedMainSource source;
  n::TransactionLimits limits;
  rd::Layout layout;
  explicit LayoutFixture(std::size_t capacity) {
    source.selection=fixture.Input().source;
    source.primary_main_count=1;source.force_packet_size=2;
    limits.inventory.max_pairs=128;limits.optimized_candidates=capacity;
    limits.sliding_entries=1024;limits.max_device_bytes=16u<<20;
    if(!rd::MakeLayout(source,limits,0,layout))throw std::runtime_error("Diagnostic fixture layout failed");
  }
};
struct Input {
  rd::Control seed=Seed();
  n::units_detail::Factors units{};
  std::vector<n::NativeFrictionResult> response;
  std::vector<std::uint32_t> flags,slots;
  explicit Input(std::size_t count):response(count),flags(count,1),slots(count) {
    units.energy=1;std::iota(slots.begin(),slots.end(),0u);
    for(std::size_t i=0;i<count;++i) {
      response[i].contact_active=i%3!=0;
      response[i].normal.elastic_energy=double(i%7)*.125;
      response[i].normal.damping_work=-double(i%11)*.0625;
      response[i].friction_work=double(i%13)*.25;
    }
  }
};
} // namespace native_diagnostic_test
