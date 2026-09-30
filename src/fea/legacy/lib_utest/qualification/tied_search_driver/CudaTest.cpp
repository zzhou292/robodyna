#include "Fixture.h"
namespace tied_driver_test {
TEST(TiedSearchDriverCuda, AllAxesKeepCompleteOrderedChoicesAndNoDuplicatePairs) {
  Fixture f;
  ts::SearchDriverResult reference;
  ASSERT_TRUE(ts::AssessSearch(f.Input(), {}, reference));
  ASSERT_EQ(reference.rows.size(), f.secondaries.size());
  EXPECT_TRUE(reference.rows[0].choice.matched);
  EXPECT_FALSE(reference.rows[1].choice.matched);
  EXPECT_FALSE(reference.rows[2].choice.matched);
  EXPECT_EQ(reference.rows[0].choice.ordered_master, 1u);
  for (unsigned axis = 0; axis < 3; ++axis) {
    ts::SearchDriverLimits limits;
    limits.axis = axis;
    ts::SearchDriverResult result;
    ASSERT_TRUE(ts::AssessSearch(f.Input(), limits, result));
    ASSERT_EQ(result.rows.size(), reference.rows.size());
    EXPECT_EQ(result.pair_count, reference.pair_count);
    for (std::size_t s = 0; s < result.rows.size(); ++s) SameChoice(result.rows[s], reference.rows[s]);
  }
}
TEST(TiedSearchDriverCuda, PairAndLateInputFailuresPreservePriorResultAndExactRetry) {
  Fixture f;
  ts::SearchDriverResult result;
  ASSERT_TRUE(ts::AssessSearch(f.Input(), {}, result));
  const auto expected = result;
  const auto* saved = result.rows.data();
  ts::SearchDriverLimits tight;
  tight.max_pairs = 1;
  EXPECT_EQ(ts::AssessSearch(f.Input(), tight, result).status, ts::SearchDriverStatus::ResourceLimit);
  EXPECT_EQ(result.rows.data(), saved);
  f.masters.back().nodes[3] = 999;
  EXPECT_EQ(ts::AssessSearch(f.Input(), {}, result).status, ts::SearchDriverStatus::InvalidInput);
  EXPECT_EQ(result.rows.data(), saved);
  f.masters.back().nodes[3] = 3;
  ASSERT_TRUE(ts::AssessSearch(f.Input(), {}, result));
  ASSERT_EQ(result.rows.size(), expected.rows.size());
  for (std::size_t s = 0; s < result.rows.size(); ++s) SameChoice(result.rows[s], expected.rows[s]);
}
}
