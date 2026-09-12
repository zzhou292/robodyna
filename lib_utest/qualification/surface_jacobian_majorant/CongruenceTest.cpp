// SPDX-License-Identifier: MIT
#include "ExactOracle.h"
#include <gtest/gtest.h>

namespace majorant_test {
TEST(SurfaceJacobianMajorant, ExactBodyAndFreeNodeCongruenceOfRepresentedMatrices) {
  // Two three-node rigid velocity maps plus two ordinary nodes, with unequal
  // lever arms and selected partial constraints. These are represented value
  // matrices, not owner/CIN data or a new rigid response implementation.
  ct::RepresentedJacobianTerm terms[8];
  for (unsigned i = 0; i < 8; ++i) {
    const double sign = i < 3 || i == 6 ? 1 : -1;
    terms[i] = {i, {sign * .125, sign * .25, -sign * .0625},
        static_cast<std::uint8_t>(i == 1 ? 2 : i == 7 ? 6 : 0)};
  }
  ct::SurfaceJacobianMajorant packet;
  ASSERT_EQ(ct::BuildRepresentedJacobianMajorant(terms, 8, 8, 123456, &packet), ct::SurfaceMajorantStatus::Ok);
  ASSERT_TRUE(ExactBounds(packet));
  constexpr unsigned columns = 18; // Two 6-DOF bodies, two free 3-DOF nodes.
  double transform[24 * columns]{};
  const ct::Vec3 arms[]{{1, 2, -3}, {-4, 5, 6}, {7, -8, 9},
      {1e3, .25, -.5}, {-2, 3, -4}, {5, 6, 7}};
  for (unsigned i = 0; i < 6; ++i) {
    const unsigned first = i < 3 ? 0 : 6;
    // Represented inverse-square-root mass scales and body-axis angular scales.
    const double translation = i < 3 ? .5 : .25;
    const double angular[]{.25, .5, 1};
    for (unsigned c = 0; c < 3; ++c) transform[(3 * i + c) * columns + first + c] = translation;
    const auto r = arms[i];
    const double cross[3][3]{{0, r.z, -r.y}, {-r.z, 0, r.x}, {r.y, -r.x, 0}};
    for (unsigned c = 0; c < 3; ++c)
      for (unsigned axis = 0; axis < 3; ++axis)
        transform[(3 * i + c) * columns + first + 3 + axis] = cross[c][axis] * angular[axis];
  }
  for (unsigned i = 6; i < 8; ++i)
    for (unsigned c = 0; c < 3; ++c) transform[(3 * i + c) * columns + 12 + 3 * (i - 6) + c] = .5;
  for (unsigned probe = 0; probe < 48; ++probe) {
    double generalized[columns];
    for (unsigned j = 0; j < columns; ++j)
      generalized[j] = (static_cast<int>((13 * j + 7 * probe) % 37) - 18) * .125;
    EXPECT_TRUE(ExactCongruence(packet, transform, columns, generalized));
  }
  // Congruence also survives rows with a common center/lever arm and exact
  // opposite contributions; cancellation does not justify guessing member mass.
  for (unsigned i = 3; i < 6; ++i)
    for (unsigned c = 0; c < 3; ++c)
      for (unsigned j = 0; j < columns; ++j)
        transform[(3 * i + c) * columns + j] = transform[(3 * (i - 3) + c) * columns + j];
  const double generalized[columns]{1, -2, 3, -4, 5, -6, 7, -8, 9};
  EXPECT_TRUE(ExactCongruence(packet, transform, columns, generalized));
}
} // namespace majorant_test
