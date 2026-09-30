// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Copyright (c) 2026 projectchrono.org
// Use of this source code is governed by the BSD-style license in LICENSE.
// =============================================================================
#include <gtest/gtest.h>
#include "chrono_vsg/utils/ChMutableMeshDraw.h"

namespace {
auto Draw(std::size_t count) {
    auto draw = vsg::VertexIndexDraw::create();
    auto indices = vsg::uintArray::create(count);
    for (std::size_t i = 0; i < count; ++i)
        (*indices)[i] = static_cast<std::uint32_t>(i);
    draw->assignIndices(indices);
    draw->indexCount = static_cast<std::uint32_t>(count);
    draw->instanceCount = 1;
    return draw;
}
}

TEST(VSGMutableMeshDraw, RemovalEmptyFrameAndBackwardSeekReuseTheSameAllocation) {
    auto draw = Draw(9);
    auto group = vsg::Group::create();
    group->addChild(draw);
    const auto backing = draw->indices;
    chrono::vsg3d::MutableMeshDraw topology;
    ASSERT_TRUE(topology.Initialize(*group, 9));
    for (std::size_t faces : {3u, 1u, 0u, 2u, 3u}) {
        ASSERT_TRUE(topology.Publish(faces));
        EXPECT_EQ(draw->indexCount, 3 * faces);
        EXPECT_EQ(draw->indices, backing);
        EXPECT_EQ(topology.capacity(), 9u);
    }
    EXPECT_FALSE(topology.Initialize(*group, 9));
}

TEST(VSGMutableMeshDraw, OversizedAndOverflowingCountsPreserveTheVisibleDraw) {
    auto draw = Draw(6);
    chrono::vsg3d::MutableMeshDraw topology;
    ASSERT_TRUE(topology.Initialize(*draw, 6));
    ASSERT_TRUE(topology.Publish(1));
    for (auto count : {std::size_t{3}, std::numeric_limits<std::size_t>::max()}) {
        EXPECT_FALSE(topology.Accepts(count));
        EXPECT_FALSE(topology.Publish(count));
        EXPECT_EQ(draw->indexCount, 3u);
    }
}

TEST(VSGMutableMeshDraw, OnlyOneCompleteSequentialTriangleDrawCanBind) {
    auto draw = Draw(6);
    chrono::vsg3d::MutableMeshDraw topology;
    EXPECT_FALSE(topology.Initialize(*draw, 0));
    EXPECT_FALSE(topology.Initialize(*draw, 5));
    EXPECT_FALSE(topology.Initialize(*draw, 9));
    draw->firstIndex = 3;
    EXPECT_FALSE(topology.Initialize(*draw, 6));
    draw->firstIndex = 0; draw->vertexOffset = 1;
    EXPECT_FALSE(topology.Initialize(*draw, 6));
    draw->vertexOffset = 0; draw->instanceCount = 2;
    EXPECT_FALSE(topology.Initialize(*draw, 6));
    draw->instanceCount = 1; draw->firstInstance = 1;
    EXPECT_FALSE(topology.Initialize(*draw, 6));
    draw->firstInstance = 0;
    auto indices = draw->indices->data.cast<vsg::uintArray>();
    (*indices)[5] = 0;
    EXPECT_FALSE(topology.Initialize(*draw, 6));
    (*indices)[5] = 5;
    auto group = vsg::Group::create();
    group->addChild(draw);
    group->addChild(Draw(6));
    EXPECT_FALSE(topology.Initialize(*group, 6));
    EXPECT_FALSE(topology.initialized());
    EXPECT_FALSE(topology.Publish(0));
    EXPECT_EQ(draw->indexCount, 6u);
    ASSERT_TRUE(topology.Initialize(*draw, 6));
}
