#include "robodyna/simulation/RbSystemSMC.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"

#include <gtest/gtest.h>

#include <memory>

namespace {
TEST(VehicleNativeModule, NewTerrainUsesTheExistingSharedWorldFrame) {
    EXPECT_TRUE(chrono::vehicle::ChWorldFrame::IsISO());
    chrono::vehicle::FlatTerrain terrain(1.25, .6f);
    const auto point = terrain.GetPoint(chrono::ChVector3d(2, 9, 3));
    EXPECT_DOUBLE_EQ(point.x(), 2);
    EXPECT_DOUBLE_EQ(point.y(), 9);
    EXPECT_DOUBLE_EQ(point.z(), 1.25);
    const auto normal = terrain.GetNormal(point);
    EXPECT_DOUBLE_EQ(normal.x(), 0);
    EXPECT_DOUBLE_EQ(normal.y(), 0);
    EXPECT_DOUBLE_EQ(normal.z(), 1);
    EXPECT_FLOAT_EQ(terrain.GetCoefficientFriction(point), .6f);
}

TEST(VehicleNativeModule, RigidPatchBelongsToTheCanonicalPhysicalSystem) {
    robodyna::simulation::RbSystemSMC system;
    system.SetNumThreads(1, 1, 1);
    chrono::vehicle::RigidTerrain terrain(&system);
    auto material = std::make_shared<chrono::ChContactMaterialSMC>();
    auto patch = terrain.AddPatch(material, chrono::ChCoordsys<>(chrono::ChVector3d(0, 0, .25)),
                                  4, 6, .5, false, 1, false);
    ASSERT_NE(patch, nullptr);
    terrain.Initialize();
    ASSERT_EQ(system.GetBodies().size(), 1u);
    EXPECT_EQ(patch->GetGroundBody(), system.GetBodies().front());
    EXPECT_EQ(patch->GetGroundBody()->GetSystem(), &system);
    EXPECT_TRUE(patch->GetGroundBody()->IsFixed());
    EXPECT_EQ(system.GetNumSteps(), 0u);
    EXPECT_EQ(system.GetVisualSystem(), nullptr);
}

TEST(VehicleNativeModule, RetainedHmmwvModelResolvesItsVehicleBaseAndConstants) {
    chrono::vehicle::hmmwv::HMMWV_RigidTire tire("native HMMWV tire", false);
    const chrono::vehicle::ChRigidTire& base = tire;
    EXPECT_EQ(base.GetName(), "native HMMWV tire");
    EXPECT_DOUBLE_EQ(base.GetRadius(), .467);
    EXPECT_DOUBLE_EQ(base.GetWidth(), .254);
    EXPECT_DOUBLE_EQ(base.GetTireMass(), 37.6);
    EXPECT_DOUBLE_EQ(base.GetTireInertia().x(), 3.84);
    EXPECT_DOUBLE_EQ(base.GetTireInertia().y(), 6.69);
    EXPECT_DOUBLE_EQ(base.GetTireInertia().z(), 3.84);
}
}  // namespace
