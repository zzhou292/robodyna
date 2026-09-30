// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../self_contact_transaction/ConeDirectionOracle.h"
#include "lib_src/collision/fixed_triangle_features/ExactPredicates.h"

namespace exact_storage_test {
namespace q = cone_direction_test;
namespace c = tlfea::contact;
namespace exact = c::fixed_triangle_features::exact;
inline int Sign(const q::Rational& value) { return value > 0 ? 1 : value < 0 ? -1 : 0; }
inline int Orient2D(c::Vec3 a, c::Vec3 b, c::Vec3 d, unsigned dropped) {
  const auto n = q::Cross(q::Subtract(q::Exact(b), q::Exact(a)),
                          q::Subtract(q::Exact(d), q::Exact(a)));
  return dropped == 1 ? -Sign(n[1]) : Sign(n[dropped]);
}
inline int Orient3D(c::Vec3 a, c::Vec3 b, c::Vec3 d, c::Vec3 origin) {
  return Sign(q::Dot(q::Subtract(q::Exact(a), q::Exact(origin)),
      q::Cross(q::Subtract(q::Exact(b), q::Exact(origin)),
               q::Subtract(q::Exact(d), q::Exact(origin)))));
}
inline int Directed(c::Vec3 a, c::Vec3 b, c::Vec3 d, c::Vec3 direction) {
  return Sign(q::Dot(q::Cross(q::Subtract(q::Exact(b), q::Exact(a)),
      q::Subtract(q::Exact(d), q::Exact(a))), q::Exact(direction)));
}
inline exact::ClosestTriangleStratum Closest(c::Vec3 point, const c::Vec3 (&triangle)[3]) {
  // Reuse the existing StratumTest Ericson partition with unlimited rationals.
  // Decimal100's finite precision is not an oracle for the full binary64 range.
  const auto p = q::Exact(point), a = q::Exact(triangle[0]);
  const auto b = q::Exact(triangle[1]), c = q::Exact(triangle[2]);
  const auto ab = q::Subtract(b, a), ac = q::Subtract(c, a);
  const auto d1 = q::Dot(ab, q::Subtract(p, a)), d2 = q::Dot(ac, q::Subtract(p, a));
  const auto d3 = q::Dot(ab, q::Subtract(p, b)), d4 = q::Dot(ac, q::Subtract(p, b));
  const auto d5 = q::Dot(ab, q::Subtract(p, c)), d6 = q::Dot(ac, q::Subtract(p, c));
  using K = exact::ClosestStratumKind;
  if (d1 <= 0 && d2 <= 0) return {K::Vertex, 0};
  if (d3 >= 0 && d4 <= d3) return {K::Vertex, 1};
  if (d1*d4 - d3*d2 <= 0 && d1 >= 0 && d3 <= 0) return {K::Edge, 0};
  if (d6 >= 0 && d5 <= d6) return {K::Vertex, 2};
  if (d5*d2 - d1*d6 <= 0 && d2 >= 0 && d6 <= 0) return {K::Edge, 2};
  if (d3*d6 - d5*d4 <= 0 && d4 >= d3 && d5 >= d6) return {K::Edge, 1};
  return {K::Face, 0};
}
}  // namespace exact_storage_test
