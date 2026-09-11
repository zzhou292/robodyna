#include "Fixture.h"
#include "NativeOracle.h"
#include "native/Packet.h"
#include <cmath>
#include <limits>
namespace tied_finalization_test {
namespace {
void Check(Fixture& f) {
  ts::FinalizedSearch output;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(), &output));
  Compare(f.Input(), *output.data(), Native(f.Input()));
}
}
TEST(TiedFinalizationNative, CompleteI2TID3StableRemovalAndRepeatedTriangleSlots) {
  Fixture f;
  f.choices[1].matched = false;
  f.choices[1].ordered_master = 0;
  f.choices[2].projection.s = 1.6;
  f.choices[3].projection.t = -1.1;
  Check(f);
}
TEST(TiedFinalizationNative, BothExactBoundariesAndEveryOriginalSlaveDisposition) {
  Fixture f;
  for (double boundary : {1.02, -1.02, 1.5, -1.5}) {
    SCOPED_TRACE(boundary);
    for (unsigned axis = 0; axis < 2; ++axis) {
      SCOPED_TRACE(axis);
      for (std::size_t s = 0; s < f.choices.size(); ++s) {
        auto& p = f.choices[s].projection;
        p.s = p.t = -0.0;
        const double value = s == 0 ? boundary : std::nextafter(boundary,
            s == 1 ? 0.0 : std::copysign(std::numeric_limits<double>::infinity(), boundary));
        (axis ? p.t : p.s) = value;
      }
      Check(f);
    }
  }
}
TEST(TiedFinalizationNative, AllUnmatchedIncludesAccumulationFlushAndDirectMessage) {
  Fixture f;
  for (auto& c : f.choices) {
    c.matched = false;
    c.ordered_master = 0;
  }
  Check(f);
}
TEST(TiedFinalizationNative, NativeDminClearsOnlyAliasedPrefixBeforeAnyCompaction) {
  Fixture f;
  const auto out = Native(f.Input(), 1);
  ASSERT_EQ(out.status, 0);
  ASSERT_EQ(out.counts[0], int(f.slaves.size()));
  for (std::size_t i = 0; i < out.dpara.size(); ++i)
    EXPECT_EQ(out.dpara[i], i < f.slaves.size() ? 0.0 : 1001.0+i);
  for (auto x : out.nmas) EXPECT_EQ(x, 0);
}
TEST(TiedFinalizationNative, InvalidLastRankAndEventOverflowPreserveAllOutputsThenRetry) {
  Fixture f;
  const auto unchanged = [](const NativeResult& out) {
    ASSERT_EQ(out.status, 1);
    for (auto x : out.counts) EXPECT_EQ(x, -17);
    for (const auto* v : {&out.nsv, &out.msr, &out.selected, &out.irupt})
      for (auto x : *v) EXPECT_EQ(x, -17);
    for (const auto* v : {&out.st, &out.stb, &out.dpara, &out.nmas})
      for (auto x : *v) EXPECT_EQ(x, -17);
    for (const auto& event : out.events) for (auto x : event) EXPECT_EQ(x, -17);
    for (const auto& event : out.event_values) for (auto x : event) EXPECT_EQ(x, -17);
  };
  f.choices.back().ordered_master = 3;
  unchanged(Native(f.Input()));
  f.choices.back().ordered_master = 2;
  unchanged(Native(f.Input(), 0, 1));
  Check(f);
}
TEST(TiedFinalizationNative, ExactLecintDistinguishesOrdinaryUniqueDuplicateAndMinusOneProfiles) {
  std::array<int,4> nodes{{4,1,3,2}}, counts{};
  int multi = -17, status = -1;
  native_tied_connection_count(4,4,nodes.data(),2,counts.data(),&multi,&status);
  ASSERT_EQ(status,0);
  EXPECT_EQ(multi,0);
  EXPECT_EQ(counts,(std::array<int,4>{{1,1,1,1}}));
  nodes[3] = 4;
  native_tied_connection_count(4,4,nodes.data(),2,counts.data(),&multi,&status);
  ASSERT_EQ(status,0);
  EXPECT_EQ(multi,1);
  EXPECT_EQ(counts,(std::array<int,4>{{1,0,1,2}}));
  native_tied_connection_count(4,4,nodes.data(),-1,counts.data(),&multi,&status);
  ASSERT_EQ(status,0);
  EXPECT_EQ(multi,4);
  EXPECT_EQ(counts,(std::array<int,4>{{0,0,0,0}}));
}
}
