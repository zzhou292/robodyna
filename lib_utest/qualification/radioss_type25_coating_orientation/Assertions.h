// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include "lib_src/math/ScalarBits.h"
#include <gtest/gtest.h>
namespace coating_orientation_test {
inline void Same(const s::NativeCoatingOrientationResult& a, const s::NativeCoatingOrientationResult& b) {
    EXPECT_EQ(a.orientation, b.orientation);
    EXPECT_TRUE(tl::math::SameScalarBits(a.center_triangle_determinant, b.center_triangle_determinant));
}
inline void SameInput(const s::NativeCoatingOrientationInput& a, const s::NativeCoatingOrientationInput& b) {
    EXPECT_EQ(a.node_count, b.node_count);
    for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(a.segment_slots[i], b.segment_slots[i]);
    for (unsigned i = 0; i < 20; ++i) {
        EXPECT_TRUE(tl::math::SameScalarBits(a.positions[i].x, b.positions[i].x));
        EXPECT_TRUE(tl::math::SameScalarBits(a.positions[i].y, b.positions[i].y));
        EXPECT_TRUE(tl::math::SameScalarBits(a.positions[i].z, b.positions[i].z));
    }
}
}
