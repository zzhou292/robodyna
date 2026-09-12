#include "Fixture.h"
#include "FrozenStability.h"

namespace tl::fea::reset_test {
TEST(NodalResetRows, SerialApiMatchesCompleteFrozenResetAndRejectsBeforeWrites) {
  EXPECT_EQ(stability::ResetRows(nullptr, 7, 10), tlfea::contact::Status::kInvalidArgument);
  for (std::uint32_t n : {1u, 255u, 256u, 257u, 4097u}) {
    for (unsigned fault = 0; fault != 12; ++fault) {
      for (std::uint64_t attempt : {0ull, 10ull}) {
        SCOPED_TRACE(n);
        SCOPED_TRACE(fault);
        SCOPED_TRACE(attempt);
        const auto seed = Seed(n);
        auto av = seed, bv = seed;
        auto a = Rows(av.data(), n, fault), b = Rows(bv.data(), n, fault);
        const auto pointers = a;
        const auto actual = stability::ResetRows(&a, 7, attempt);
        const auto old = reset_frozen_stability::ResetRows(&b, 7, attempt);
        EXPECT_EQ(actual, old); SameRows(a, b); SameBytes(av, bv);
        EXPECT_EQ(a.stiffness, pointers.stiffness); EXPECT_EQ(a.damping, pointers.damping);
        CheckCleared(seed, av, n, actual == tlfea::contact::Status::kOk);
      }
    }
  }
}
TEST(NodalResetRows, HeaderPublicationWaitsForClearAndEpochRetryRetainsContract) {
  auto values = Seed(257);
  auto rows = Rows(values.data(), 257, 0);
  const auto original = rows;
  const auto seed = values;
  ASSERT_EQ(stability::detail::BeginResetRows(&rows, 7, 10), tlfea::contact::Status::kOk);
  SameBytes(values, seed);
  EXPECT_FALSE(rows.valid); EXPECT_EQ(rows.base_epoch, original.base_epoch);
  EXPECT_EQ(rows.attempt, original.attempt); EXPECT_EQ(rows.sealed, original.sealed);
  std::fill(rows.stiffness, rows.stiffness + rows.node_count, 0.);
  std::fill(rows.damping, rows.damping + rows.node_count, 0.);
  ASSERT_EQ(stability::detail::CompleteResetRows(&rows, 7, 10), tlfea::contact::Status::kOk);
  EXPECT_TRUE(rows.valid); EXPECT_TRUE(rows.initialized); EXPECT_FALSE(rows.sealed);
  EXPECT_EQ(rows.base_epoch, 7u); EXPECT_EQ(rows.attempt, 10u);
  values[1] = 4;
  EXPECT_EQ(stability::ResetRows(&rows, 7, 10), tlfea::contact::Status::kStaleTrial);
  EXPECT_FALSE(rows.valid); EXPECT_EQ(values[1], 4.);
  ASSERT_EQ(stability::ResetRows(&rows, 7, 11), tlfea::contact::Status::kOk);
  values[1] = 6;
  ASSERT_EQ(stability::ResetRows(&rows, 8, 1), tlfea::contact::Status::kOk);
  EXPECT_EQ(Bits(values[1]), Bits(0.));
}
} // namespace tl::fea::reset_test
