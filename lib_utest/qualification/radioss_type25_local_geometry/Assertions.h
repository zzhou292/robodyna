// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <gtest/gtest.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
namespace type25_geometry_test {
inline void Number(double a, double b, bool exact) {
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b));
  if (exact) {
    std::uint64_t x, y; std::memcpy(&x, &a, 8); std::memcpy(&y, &b, 8); EXPECT_EQ(x, y);
  } else EXPECT_NEAR(a, b, 64 * std::numeric_limits<double>::epsilon() *
      std::max({std::abs(a), std::abs(b), 1e-300}));
}
template<class A, class B> inline void Same(const n::RawGeometryResult<A>& a,
    const n::RawGeometryResult<B>& b, bool exact = false) {
  EXPECT_EQ(a.key.secondary_source_id, b.key.secondary_source_id);
  EXPECT_EQ(a.key.generation, b.key.generation);
  EXPECT_EQ(a.key.history_index, b.key.history_index); EXPECT_EQ(a.key.main_segment, b.key.main_segment);
  EXPECT_EQ(a.selection_code, b.selection_code);
  const double x[]{a.normal.x,a.normal.y,a.normal.z,a.weights[0],a.weights[1],a.weights[2],a.weights[3],
      a.geometric_penetration,a.gap,a.distance,a.incoming_stiffness};
  const double y[]{b.normal.x,b.normal.y,b.normal.z,b.weights[0],b.weights[1],b.weights[2],b.weights[3],
      b.geometric_penetration,b.gap,b.distance,b.incoming_stiffness};
  for (unsigned i = 0; i < 11; ++i) { SCOPED_TRACE(i); Number(x[i], y[i], exact); }
}
inline void Same(const n::NativeGeometryHistory& a,
    const n::NativeGeometryHistory& b, bool exact = true) {
  EXPECT_EQ(a.secondary_source_id, b.secondary_source_id); EXPECT_EQ(a.generation, b.generation);
  const auto& x = a.row; const auto& y = b.row;
  for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(x.irtlm[i], y.irtlm[i]);
  const auto& p=x.history.normal; const auto& q=y.history.normal;
  const double first[]{p.previous_penetration,p.previous_stiffness,p.staged_penetration,
    p.staged_stiffness,p.damping_half_force,x.history.previous_force.x,x.history.previous_force.y,
    x.history.previous_force.z,x.history.staged_force.x,x.history.staged_force.y,x.history.staged_force.z,
    x.penetration_auxiliary,x.penetration_offset,x.selection_metric[0],x.selection_metric[1]};
  const double second[]{q.previous_penetration,q.previous_stiffness,q.staged_penetration,
    q.staged_stiffness,q.damping_half_force,y.history.previous_force.x,y.history.previous_force.y,
    y.history.previous_force.z,y.history.staged_force.x,y.history.staged_force.y,y.history.staged_force.z,
    y.penetration_auxiliary,y.penetration_offset,y.selection_metric[0],y.selection_metric[1]};
  for (unsigned i=0;i<15;++i) { SCOPED_TRACE(i); Number(first[i],second[i],exact); }
}
template<class U> inline n::RawGeometryResult<U> Sentinel() {
  n::RawGeometryResult<U> r; r.key={71,72,73,74};r.selection_code=4;
  r.normal={11,12,13}; for (unsigned i=0;i<4;++i) r.weights[i]=14+i;
  r.geometric_penetration=18;r.gap=19;r.distance=20;r.incoming_stiffness=21;return r;
}
} // namespace type25_geometry_test
