#include "../LoadedWall.h"
#include "output/full_shell/FixedStepHorizon.h"
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
    for (const auto duration : {.0005,.00135,.005,.02,.05}) {
        settings.requested_duration_s=duration;
        const auto placement=Place({{{-2,-1,.05},{2,1,1.7}}},settings);
        EXPECT_GT(placement.nominal_forward_travel.lower,previous);
        EXPECT_EQ(config.startup.reserved_step_s,3e-7);
        EXPECT_LE(placement.declared_world_envelope.minimum.y,-1.25);
        EXPECT_GE(placement.declared_world_envelope.maximum.y,1.25);
        previous=placement.nominal_forward_travel.upper;
    }
    for (const auto bad : {0.0,-.005,.051,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        settings.requested_duration_s=bad;
        EXPECT_THROW(CheckSettings(settings),std::runtime_error);
    }
}
TEST(VehicleLoadedWallValues, FixedHorizonSharesArchiveBoundaryAndPreservesRejectedOutput) {
    namespace records=output::full_shell;
    const double durations[]={.0005,.005,.02,.05};
    const std::uint64_t expected[]={1667,16667,66667,166667};
    for(unsigned i=0;i<4;++i) {
        std::uint64_t count=0;
        ASSERT_TRUE(records::PlanFixedStepHorizon(3e-7,durations[i],count));
        EXPECT_EQ(count,expected[i]);
        EXPECT_TRUE(records::MatchesFixedStepHorizon(count,3e-7,durations[i]));
        EXPECT_FALSE(records::MatchesFixedStepHorizon(count-1,3e-7,durations[i]));
        EXPECT_FALSE(records::MatchesFixedStepHorizon(count+1,3e-7,durations[i]));
    }
    const double h=1.0/1024;
    std::uint64_t count=0;
    ASSERT_TRUE(records::PlanFixedStepHorizon(h,5*h,count));
    EXPECT_EQ(count,5u);
    ASSERT_TRUE(records::PlanFixedStepHorizon(h,std::nextafter(5*h,6*h),count));
    EXPECT_EQ(count,6u);
    for(const double invalid:{0.0,-1.0,std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::denorm_min()}) {
        count=123;
        EXPECT_FALSE(records::PlanFixedStepHorizon(invalid,.005,count));
        EXPECT_EQ(count,123u);
    }
}
} // namespace crash::cases::vehicle_wall::test
