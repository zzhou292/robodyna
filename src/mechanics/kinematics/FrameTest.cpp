#include <gtest/gtest.h>
#include "chrono/core/ChRotation.h"
#include "chrono/physics/ChBodyFrame.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/utils/ChConstants.h"

#include <sstream>

namespace {
// A frame with a numerical variable block; no rigid body, FE node or system.
class Frame : public chrono::ChBodyFrame {
  public:
    chrono::ChVariables& Variables() override { return variables_; }
  private:
    chrono::ChVariablesBodyOwnMass variables_;
};
}

TEST(NeutralFrame, InverseAndEquivalentForceRepresentations) {
    Frame frame;
    frame.SetPos({4, -2, 1});
    frame.SetRot(chrono::QuatFromAngleZ(chrono::CH_PI_2));
    const chrono::ChVector3d point(2, 0, 0), force(0, 3, 0);
    const auto point_world = frame.TransformPointLocalToParent(point);
    const auto force_world = frame.TransformDirectionLocalToParent(force);
    EXPECT_LT((frame.TransformPointParentToLocal(point_world) - point).Length(), 1e-14);
    const auto local = frame.AppliedForceLocalToWrenchParent(force, point);
    const auto world = frame.AppliedForceParentToWrenchParent(force_world, point_world);
    EXPECT_LT((local.force - chrono::ChVector3d(-3, 0, 0)).Length(), 1e-14);
    EXPECT_LT((local.torque - chrono::ChVector3d(0, 0, 6)).Length(), 1e-14);
    EXPECT_LT((world.force - local.force).Length(), 1e-14);
    EXPECT_LT((world.torque - local.torque).Length(), 1e-14);
}

TEST(NeutralFrame, AppliedWrenchPreservesInstantaneousPower) {
    Frame frame;
    frame.SetPos({-3, 4, 2});
    frame.SetRot(chrono::QuatFromAngleY(.37));
    frame.SetPosDt({1, 2, -1});
    const chrono::ChVector3d omega(.5, -.25, 2), point(.4, -.2, .7), force(2, -3, .25);
    frame.SetAngVelParent(omega);
    const auto wrench = frame.AppliedForceLocalToWrenchParent(force, point);
    const auto point_velocity = frame.PointSpeedLocalToParent(point);
    EXPECT_NEAR(chrono::Vdot(wrench.force, point_velocity),
                chrono::Vdot(wrench.force, frame.GetPosDt()) + chrono::Vdot(wrench.torque, omega), 1e-12);
}

TEST(NeutralFrame, MovingFrameArchivePreservesPoseAndVelocity) {
    Frame frame;
    frame.SetPos({1.25, -2.5, .125});
    frame.SetRot(chrono::QuatFromAngleX(.3));
    frame.SetPosDt({.5, -.25, 1});
    frame.SetAngVelParent({.2, .4, -.3});
    std::stringstream stream;
    {
        chrono::ChArchiveOutJSON output(stream);
        output << CHNVP(frame);
    }
    Frame restored;
    {
        chrono::ChArchiveInJSON input(stream);
        input >> CHNVP(restored, "frame");
    }
    EXPECT_LT((restored.GetPos() - frame.GetPos()).Length(), 1e-14);
    EXPECT_LT((restored.GetPosDt() - frame.GetPosDt()).Length(), 1e-14);
    EXPECT_LT((restored.GetAngVelParent() - frame.GetAngVelParent()).Length(), 1e-14);
    EXPECT_LT((restored.GetRotMat() - frame.GetRotMat()).norm(), 1e-14);
}
