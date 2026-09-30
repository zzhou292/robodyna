#include "SourcePartWallCheckFixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace crash::cases::source_part_wall {
namespace {
using namespace check;
TEST(SourcePartWallSetup, OriginalNativeEnergyFloorAndPlacedGapHaveIndependentEnclosures) {
    const auto f=std::make_unique<Fixture>();SourcePartWallSetup setup;
    const auto prepared=f->Setup(setup,f->settings);ASSERT_TRUE(prepared)<<prepared.message;
    const auto& c=*setup.certificate();const auto& s=*setup.settings();
    long double kinetic=0;
    for(unsigned n=0;n<source::NodeCount;++n) {
        kinetic+=.5L*f->binding.nodes()[n].native.mass;
        EXPECT_EQ(output::Bits(setup.inverse_mass()[n]),output::Bits(f->inverse[n]));
        EXPECT_EQ(setup.translation_fixed_bits()[n],0);
        EXPECT_GE(setup.source_geometry()->weights()->node(n).area.lower,s.area_floor);
    }
    EXPECT_LE(c.native_initial_kinetic.lower,kinetic);EXPECT_GE(c.native_initial_kinetic.upper,kinetic);
    EXPECT_EQ(output::Bits(c.measured_initial_kinetic),output::Bits(f->kinetic));
    EXPECT_EQ(c.initial_velocity,s.initial_velocity);EXPECT_EQ(c.fixed_dt,Step);
    EXPECT_EQ(c.minimum_area_node,116u);EXPECT_GE(c.minimum_nodal_area_lower,1e-5);
    EXPECT_GE(c.kinetic_budget_upper,static_cast<long double>(s.kinetic_budget_factor)*c.native_initial_kinetic.upper);
    const long double potential=.5L*c.stiffness_per_area*s.area_floor*s.design_penetration*s.design_penetration;
    EXPECT_LE(c.design_potential_lower,potential);EXPECT_GE(c.design_potential_lower,c.kinetic_budget_upper);
    EXPECT_TRUE(c.coverage.covered);const auto& wall=*setup.placed_wall();const auto bounds=setup.source_geometry()->reference_bounds();
    const auto wall_x=wall.geometry()->wall_x();const long double gap=static_cast<long double>(wall_x)-bounds[1].x;
    EXPECT_GT(c.leading_gap.lower,0);EXPECT_LE(c.leading_gap.lower,gap);EXPECT_GE(c.leading_gap.upper,gap);
    EXPECT_EQ(output::Bits(wall.placement()->translation_x_m),output::Bits((bounds[1].x+s.leading_gap)-.05));
    EXPECT_EQ(output::Bits(wall_x),output::Bits(.05+wall.placement()->translation_x_m));
    EXPECT_EQ(c.coverage.physical.minimum.x,wall_x);EXPECT_EQ(c.coverage.physical.maximum.x,wall_x);
    EXPECT_LE(c.coverage.physical.minimum.y,static_cast<long double>(bounds[0].y)-s.motion_margin);
    EXPECT_GE(c.coverage.physical.maximum.z,static_cast<long double>(bounds[1].z)+s.motion_margin);
    for(unsigned refinement:{2u,4u}) {
        SourcePartWallSetup refined;ASSERT_TRUE(f->Setup(refined,s,Step/refinement));
        EXPECT_EQ(output::Bits(refined.certificate()->stiffness_per_area),output::Bits(c.stiffness_per_area));
        EXPECT_EQ(refined.certificate()->fixed_dt,Step/refinement);
    }
}
TEST(SourcePartWallSetup, LateMassFloorAndMeasuredEnergyFailuresPreservePublication) {
    auto f=std::make_unique<Fixture>();SourcePartWallSetup good;ASSERT_TRUE(f->Setup(good,f->settings));
    const auto* certificate=good.certificate();const auto bytes=Bytes(*certificate);const auto* wall=good.placed_wall();
    EXPECT_EQ(f->Setup(good,f->settings).status,SourcePartWallStatus::AlreadyInitialized);
    EXPECT_EQ(good.certificate(),certificate);EXPECT_EQ(Bytes(*certificate),bytes);EXPECT_EQ(good.placed_wall(),wall);
    SourcePartWallSetup failed;
    const double inverse=f->inverse.back();f->inverse.back()=std::nextafter(inverse,HUGE_VAL);
    const auto mass=f->Setup(failed,f->settings);EXPECT_EQ(mass.status,SourcePartWallStatus::MassMismatch);EXPECT_EQ(mass.node,116u);
    EXPECT_FALSE(failed.initialized());EXPECT_EQ(failed.certificate(),nullptr);f->inverse.back()=inverse;
    auto settings=f->settings;settings.area_floor=std::nextafter(certificate->minimum_nodal_area_lower,HUGE_VAL);
    const auto floor=f->Setup(failed,settings);EXPECT_EQ(floor.status,SourcePartWallStatus::CertificateFailure);EXPECT_EQ(floor.node,116u);
    EXPECT_FALSE(failed.initialized());const double kinetic=f->kinetic;
    f->kinetic=std::nextafter(certificate->native_initial_kinetic.upper,HUGE_VAL);
    EXPECT_EQ(f->Setup(failed,f->settings).status,SourcePartWallStatus::CertificateFailure);EXPECT_FALSE(failed.initialized());
    f->kinetic=kinetic;settings=f->settings;settings.motion_margin=3;
    EXPECT_EQ(f->Setup(failed,settings).status,SourcePartWallStatus::GeometryFailure);EXPECT_FALSE(failed.initialized());
    ASSERT_TRUE(f->Setup(failed,f->settings));EXPECT_EQ(Bytes(*good.certificate()),bytes);
}
TEST(SourcePartWallSetup, InvalidInitialStampSettingsAndContactStepGuardAreStaged) {
    const auto f=std::make_unique<Fixture>();SourcePartWallSetup setup;auto stamp=f->Stamp();stamp.epoch=1;
    EXPECT_EQ(setup.Initialize(f->source,f->binding,stamp,f->inverse.data(),f->kinetic,f->wall,f->wall_bytes,f->settings).status,
              SourcePartWallStatus::InvalidInput);EXPECT_FALSE(setup.initialized());
    auto settings=f->settings;settings.configuration_id=0;
    EXPECT_EQ(f->Setup(setup,settings).status,SourcePartWallStatus::InvalidInput);
    settings=f->settings;settings.initial_velocity[1]=1;
    EXPECT_EQ(f->Setup(setup,settings).status,SourcePartWallStatus::InvalidInput);
    double upper=-7;ASSERT_TRUE(CheckSourcePartContactStep(1./64,16,.125,&upper));
    EXPECT_GE(static_cast<long double>(upper),1.L/64*std::sqrt(16.L));EXPECT_LE(upper,.125);
    const auto before=output::Bits(upper);
    EXPECT_EQ(CheckSourcePartContactStep(1./16,16,.125,&upper).status,SourcePartWallStatus::StepLimit);
    EXPECT_EQ(output::Bits(upper),before);
    EXPECT_EQ(CheckSourcePartContactStep(Step,std::numeric_limits<double>::quiet_NaN(),.125,&upper).status,SourcePartWallStatus::InvalidInput);
    EXPECT_EQ(output::Bits(upper),before);EXPECT_FALSE(CheckSourcePartContactStep(Step,16,.125,nullptr));
}
} // namespace
} // namespace crash::cases::source_part_wall
int main(int argc,char** argv) {
    if(argc<3)return 2;
    crash::cases::source_part_wall::check::SourcePath=argv[1];crash::cases::source_part_wall::check::WallPath=argv[2];
    ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
