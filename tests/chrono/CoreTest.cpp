#include <gtest/gtest.h>

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChRotation.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/utils/ChConstants.h"

#include <sstream>

TEST(ChronoHostBridge, FramePreservesPointAndInverse) {
    const chrono::ChFrame<> frame({1, 2, 3}, chrono::QuatFromAngleZ(chrono::CH_PI_2));
    const chrono::ChVector3d local(2, 0, -1);
    const auto world = frame.TransformPointLocalToParent(local);
    EXPECT_NEAR(world.x(), 1, 1e-14);
    EXPECT_NEAR(world.y(), 4, 1e-14);
    EXPECT_NEAR(world.z(), 2, 1e-14);
    EXPECT_LT((frame.TransformPointParentToLocal(world) - local).Length(), 1e-14);
}

TEST(ChronoHostBridge, CoreVectorArchiveRoundTrip) {
    std::stringstream stream;
    chrono::ChVector3d point(1.25, -2.5, .125);
    {
        chrono::ChArchiveOutJSON output(stream);
        output << CHNVP(point);
    }
    point = chrono::ChVector3d(0, 0, 0);
    {
        chrono::ChArchiveInJSON input(stream);
        input >> CHNVP(point);
    }
    EXPECT_DOUBLE_EQ(point.x(), 1.25);
    EXPECT_DOUBLE_EQ(point.y(), -2.5);
    EXPECT_DOUBLE_EQ(point.z(), .125);
}
