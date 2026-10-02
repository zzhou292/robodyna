#include "chrono_models/robot/industrial/IndustrialKinematicsSCARA.h"
#include "chrono_models/robot/industrial/IndustrialRobotSCARA.h"
#include "chrono_models/robot/industrial/TrajectoryInterpolator.h"
#include "robodyna/simulation/RbSystemNSC.h"

#include <gtest/gtest.h>

#include <cmath>

TEST(RobotModelHost, ScaraForwardInverseKinematicsAndCopyPreserveCoordinates) {
    using namespace chrono;
    const std::array<ChCoordsysd, 5> joints = {
        ChCoordsysd(ChVector3d(0, 0, 0)), ChCoordsysd(ChVector3d(0.4, 0, 0.5)),
        ChCoordsysd(ChVector3d(0.7, 0, 0.5)), ChCoordsysd(ChVector3d(0.7, 0, 0.5)),
        ChCoordsysd(ChVector3d(0.8, 0, 0.4))};
    industrial::IndustrialKinematicsSCARA kinematics(joints, {0.5, 0.4, 0.3, 0.1, 0.1});
    ASSERT_EQ(kinematics.GetNumJoints(), 4);
    ChVectorDynamic<> angles(4);
    angles << 0.2, 0.4, -0.1, -0.025;
    const auto target = kinematics.GetFK(angles);
    const auto inverse = kinematics.GetIK(target);
    for (int index = 0; index < 4; ++index)
        EXPECT_NEAR(inverse[index], angles[index], 1e-12);
    industrial::IndustrialKinematicsSCARA copied(kinematics);
    EXPECT_NEAR((copied.GetFK(inverse).pos - target.pos).Length(), 0, 1e-12);
    EXPECT_NEAR(std::abs(copied.GetFK(inverse).rot.Dot(target.rot)), 1, 1e-12);
}

TEST(RobotModelHost, RobotBodiesAndMotorsUseTheExistingSystemOwner) {
    robodyna::simulation::RbSystemNSC system;
    const std::array<double, 5> lengths = {0.5, 0.4, 0.3, 0.1, 0.1};
    chrono::industrial::IndustrialRobotSCARA robot(&system, lengths);
    EXPECT_EQ(robot.GetLengths(), lengths);
    ASSERT_EQ(robot.GetBodies().size(), 5);
    EXPECT_EQ(system.GetBodies().size(), 5);
    EXPECT_EQ(robot.GetMotors().size(), 4);
    EXPECT_EQ(robot.GetMotionFunctions().size(), 4);
    EXPECT_EQ(robot.GetMarkers().size(), 5);
    EXPECT_TRUE(robot.GetBase()->IsFixed());
    for (const auto& body : robot.GetBodies())
        EXPECT_EQ(body->GetSystem(), &system);
    EXPECT_DOUBLE_EQ(system.GetTime(), 0);
    EXPECT_EQ(system.GetVisualSystem(), nullptr);
}

TEST(RobotModelHost, JointTrajectoryKeepsOriginalDurationsAndInterpolation) {
    using Interpolator = chrono::industrial::TrajectoryInterpolatorJointSpace;
    std::vector<chrono::ChVectorDynamic<>> points(3, chrono::ChVectorDynamic<>(2));
    points[0] << 0, 0;
    points[1] << 2, -1;
    points[2] << 4, -2;
    Interpolator interpolation(4, points, Interpolator::SpacefunType::LINEAR);
    EXPECT_EQ(interpolation.GetDurations(), (std::vector<double>{2, 2}));
    EXPECT_EQ(interpolation.GetMotionTimesCumulative(), (std::vector<double>{0, 2, 4}));
    for (const double time : {1.0, 3.0}) {
        const auto value = interpolation.GetInterpolation(time);
        EXPECT_DOUBLE_EQ(value[0], time);
        EXPECT_DOUBLE_EQ(value[1], -time / 2);
    }
}
