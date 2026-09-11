#include "../LoadedWall.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::cases::vehicle_wall::test {
TEST(VehicleLoadedWallValues, ExplicitDurationsRecomputeEnvelopeWithoutChangingStep) {
    auto settings=LoadedWallSettings();
    const auto config=LoadedWallConfig();
    EXPECT_EQ(settings.mesh_profile,WallMeshProfile::EnvelopeRectangleV1);
    EXPECT_EQ(settings.transverse_margin_m,.25);
    EXPECT_EQ(config.structural.profile,tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace);
    EXPECT_TRUE(tl::fea::ValidCinStructuralStep(config.structural));
    double previous=0;
    for (const auto duration : {.005,.02,.05}) {
        settings.requested_duration_s=duration;
        const auto placement=Place({{{-2,-1,.05},{2,1,1.7}}},settings);
        EXPECT_GT(placement.nominal_forward_travel.lower,previous);
        EXPECT_EQ(config.startup.reserved_step_s,3e-7);
        EXPECT_LE(placement.declared_world_envelope.minimum.y,-1.25);
        EXPECT_GE(placement.declared_world_envelope.maximum.y,1.25);
        previous=placement.nominal_forward_travel.upper;
    }
    for (const auto bad : {0.0,-.005,.006,.051,std::numeric_limits<double>::infinity()}) {
        settings.requested_duration_s=bad;
        EXPECT_THROW(CheckSettings(settings),std::runtime_error);
    }
}
} // namespace crash::cases::vehicle_wall::test
