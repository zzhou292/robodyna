// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <array>
namespace native_diagnostic_test {
TEST(NativeDiagnosticLayout, ActualArenaSeparatesControlAndAllReadOperands) {
  for(std::size_t count:{0u,1u,257u}) {
    SCOPED_TRACE(count);LayoutFixture f(count);const auto& l=f.layout;
    const std::array<tl::util::ArenaRegion,4> reads{{l.responses,l.positive_flags,l.sorted_slots,l.row_packets}};
    ASSERT_EQ(l.control.bytes,sizeof(rd::Control));
    for(const auto& r:reads) {
      ASSERT_LE(r.offset+r.bytes,l.bytes);
      EXPECT_TRUE(!r.bytes||r.offset+r.bytes<=l.control.offset||l.control.offset+l.control.bytes<=r.offset);
    }
    std::vector<std::byte> arena(l.bytes);
    const auto d=rd::Bind(arena.data(),l,f.source,f.limits);
    EXPECT_EQ(reinterpret_cast<const std::byte*>(d.control)-arena.data(),l.control.offset);
    EXPECT_EQ(reinterpret_cast<const std::byte*>(d.responses)-arena.data(),l.responses.offset);
    EXPECT_EQ(reinterpret_cast<const std::byte*>(d.sorted_slots)-arena.data(),l.sorted_slots.offset);
    EXPECT_EQ(reinterpret_cast<const std::byte*>(d.positive_flags)-arena.data(),l.positive_flags.offset);
  }
}
TEST(NativeDiagnosticLayout, ExactDeviceAdmissionAndFailureAtomicLayout) {
  LayoutFixture f(257);const auto expected=f.layout;
  f.limits.max_device_bytes=expected.bytes;
  rd::Layout actual;ASSERT_TRUE(rd::MakeLayout(f.source,f.limits,0,actual));
  EXPECT_EQ(actual.bytes,expected.bytes);EXPECT_EQ(actual.control.offset,expected.control.offset);
  --f.limits.max_device_bytes;actual.bytes=123;actual.control={47,3,19};
  ASSERT_FALSE(rd::MakeLayout(f.source,f.limits,0,actual));
  EXPECT_EQ(actual.bytes,123u);EXPECT_EQ(actual.control.offset,47u);
  EXPECT_EQ(actual.control.count,3u);EXPECT_EQ(actual.control.bytes,19u);
}
} // namespace native_diagnostic_test
