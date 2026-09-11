// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Values.h"
#include <limits>
namespace tl::fea::cin_capture_test {
TEST(CinParallelCapture, BoundedGridCoversEveryTailRowWithExactBitsAndRigidMask) {
  for (const auto n : {1u, 127u, 128u, 129u, 272u, 33001u}) {
    SCOPED_TRACE(n);
    Values old(n), next(n);
    CopyFrozen(old.Input());
    const auto input = next.Input();
    const auto stride = cin_advance::capture::Threads*cin_advance::capture::Blocks(n);
    for (unsigned lane = 0; lane < stride; ++lane) {
      for (std::uint32_t node = lane; node < n; node += stride) {
        cin_advance::capture::CopyNode(input, node);
      }
    }
    SameValues(old.output, next.output);
    SameValues(old.work, next.work);
  }
}
TEST(CinParallelCapture, UnmaskedRowsAndNonzeroRigidMaskHaveExactConsumedReadset) {
  Values old(272), next(272);
  auto a = old.Input(), b = next.Input();
  a.groups.member_nodes = b.groups.member_nodes = nullptr;
  CopyFrozen(a);
  for (std::uint32_t node = 0; node < next.nodes; ++node) cin_advance::capture::CopyNode(b, node);
  SameValues(old.output, next.output);
  // A rigid row returns before work or capture is read, including noncanonical
  // nonzero masks. These raw packets exercise the private copy read set only.
  const std::uint8_t mask = 255;
  cin_advance::Input masked;
  masked.groups.member_nodes = &mask;
  cin_advance::capture::CopyNode(masked, 0);
}
TEST(CinParallelCapture, LaunchGridIsBoundedWithoutRoundingOverflowOrNewStorage) {
  EXPECT_EQ(cin_advance::capture::Blocks(0), 0u);
  EXPECT_EQ(cin_advance::capture::Blocks(1), 1u);
  EXPECT_EQ(cin_advance::capture::Blocks(128), 1u);
  EXPECT_EQ(cin_advance::capture::Blocks(129), 2u);
  EXPECT_EQ(cin_advance::capture::Blocks(UINT32_MAX), 256u);
  EXPECT_EQ(cin_advance::capture::Threads, 128u);
}
} // namespace tl::fea::cin_capture_test
