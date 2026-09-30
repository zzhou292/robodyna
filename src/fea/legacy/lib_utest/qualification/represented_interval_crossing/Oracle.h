// SPDX-License-Identifier: MIT
#pragma once

#include "Fixture.h"

namespace represented_interval_test {

struct ExactOracleResult {
  bool valid = false;
  bool intersects = false;
  bool coplanar = false;
  bool vertex_face = false;
  bool edge_edge = false;
};

// Independent exact-rational static oracle.  Time is numerator/denominator;
// unlike production it does not use dyadic integer/exponent predicates.
ExactOracleResult ExactOracleAt(
    const ct::RepresentedTrianglePath& a,
    const ct::RepresentedTrianglePath& b, std::uint64_t numerator,
    std::uint64_t denominator);

}  // namespace represented_interval_test
