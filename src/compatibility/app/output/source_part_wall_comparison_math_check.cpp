#include "SourcePartWallComparisonMath.h"
#include <gtest/gtest.h>
#include <limits>

namespace crash::output::wall_comparison {
namespace {
Document Frame() {
    Document d;d.SetObject();Number(d,"accepted_time_s",0);
    std::array<double,351> vectors{};std::array<double,468> orientation{};
    for(unsigned n=0;n<117;++n) orientation[4*n]=1;
    for(const char* name:{"position_xyz_m","synchronized_velocity_xyz_m_per_s","synchronized_omega_world_xyz_rad_per_s"})
        FiniteArray(d,name,vectors.data(),vectors.size());
    FiniteArray(d,"orientation_wxyz",orientation.data(),orientation.size());return d;
}
TEST(SourceWallComparisonMath, NativeContactEnergyAndFrozenContactScalesAreDistinct) {
    const auto a=Frame(),b=Frame();const auto scale=MakeScales(.125,.25,1,.000375);
    const ContactSample ca{.125,0,0,0,0},cb{.1,.015,.02,.005,10};
    const auto difference=Difference(a,ca,b,cb,scale);
    for(unsigned n=0;n<4;++n) EXPECT_EQ(difference[n],0);
    EXPECT_NEAR(difference[4],.1,1e-15); // (.1+.015+.02-.125)/.1 J.
    EXPECT_DOUBLE_EQ(difference[5],.16); EXPECT_DOUBLE_EQ(difference[6],.02);
    EXPECT_DOUBLE_EQ(difference[7],10/(2*.125/.000375));
    EXPECT_THROW(MakeScales(0,.25,1,.000375),std::runtime_error);
    auto invalid=scale;invalid.impulse=0;
    EXPECT_THROW(Difference(a,ca,b,cb,invalid),std::runtime_error);
    auto negative=cb;negative.potential=-1;
    EXPECT_THROW(Difference(a,ca,b,negative,scale),std::runtime_error);
}
TEST(SourceWallComparisonMath, FixedContractionRejectsFineGrowthAndNonfiniteValues) {
    EXPECT_TRUE(Converged(.10,.05));EXPECT_TRUE(Converged(0,1e-6));
    EXPECT_FALSE(Converged(.10001,.05));EXPECT_FALSE(Converged(.10,.05001));
    EXPECT_FALSE(Converged(.04,.033));EXPECT_FALSE(Converged(-1,0));
    EXPECT_FALSE(Converged(0,std::numeric_limits<double>::quiet_NaN()));
}
TEST(SourceWallComparisonMath, EventWindowsUseOneSamplingSlackAndExplicitCertificateUncertainty) {
    const EventBracket a{true,8*BaseStep,9*BaseStep,0};
    EventBracket b{true,10*BaseStep,11*BaseStep,0};
    EXPECT_TRUE(EventsAgree(a,b,BaseStep));
    b.lower=11*BaseStep;b.upper=12*BaseStep;EXPECT_FALSE(EventsAgree(a,b,BaseStep));
    b.uncertainty=BaseStep;EXPECT_TRUE(EventsAgree(a,b,BaseStep));
    b.observed=false;EXPECT_FALSE(EventsAgree(a,b,BaseStep));
    EXPECT_TRUE(EventsAgree({}, {},BaseStep));
    b.upper=b.lower-BaseStep;EXPECT_THROW(EventsAgree(a,b,BaseStep),std::runtime_error);
}
} // namespace
} // namespace crash::output::wall_comparison
