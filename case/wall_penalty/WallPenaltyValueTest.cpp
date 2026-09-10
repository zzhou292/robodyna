#include "UniformTranslationKinetic.h"
#include "WallPenaltyCertification.h"
#include "WallPlacementBounds.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>
namespace crash::cases::wall_penalty {
namespace {
template<class T> auto Bytes(const T& value) {
    std::array<unsigned char,sizeof(T)> bytes;std::memcpy(bytes.data(),&value,sizeof(value));return bytes;
}
TEST(WallPenaltyValues, ExplicitUniformMassMetricEnclosesOrderedPositiveInputsAndRejectsLateOverflow) {
    UniformTranslationKinetic value;ASSERT_TRUE(BeginUniformTranslation(8,&value));
    const double masses[]{.3,1e-17,.4,1e-17,.25};long double truth=0;double nominal=0;
    for(double mass:masses) { ASSERT_TRUE(AddTranslationMass(mass,&value));truth+=.5L*mass*8*8;nominal+=.5*mass*(8*8); }
    contact::Q4CertifiedIntegral output;ASSERT_TRUE(FinishUniformTranslation(value,&output));
    EXPECT_EQ(output.value,nominal);EXPECT_LE(output.lower,truth);EXPECT_GE(output.upper,truth);
    const auto before=Bytes(value);
    EXPECT_FALSE(AddTranslationMass(std::numeric_limits<double>::max(),&value));EXPECT_EQ(Bytes(value),before);
    EXPECT_FALSE(AddTranslationMass(0,&value));EXPECT_EQ(Bytes(value),before);
    EXPECT_FALSE(BeginUniformTranslation(std::numeric_limits<double>::quiet_NaN(),&value));EXPECT_EQ(Bytes(value),before);
    UniformTranslationKinetic empty;const auto prior=Bytes(output);
    EXPECT_FALSE(FinishUniformTranslation(empty,&output));EXPECT_EQ(Bytes(output),prior);
}
TEST(WallPenaltyValues, GapMotionAndLocalRateRejectUnrepresentableOrMalformedValuesWithoutPublication) {
    const std::array<contact::Vec3,2> bounds{{{-.17,-.5,.01},{-.1,.1,.05}}};
    contact::PlanarWallBox box;ASSERT_TRUE(ExpandProjectedMotion(bounds,.02,&box));
    EXPECT_EQ(box.minimum.x,bounds[0].x);EXPECT_EQ(box.maximum.x,bounds[1].x);
    EXPECT_LE(box.minimum.y,static_cast<long double>(bounds[0].y)-.02);
    EXPECT_GE(box.maximum.z,static_cast<long double>(bounds[1].z)+.02);
    const auto before=Bytes(box);EXPECT_FALSE(ExpandProjectedMotion(bounds,-.02,&box));EXPECT_EQ(Bytes(box),before);
    auto invalid=bounds;invalid[1].y=std::numeric_limits<double>::max();
    EXPECT_FALSE(ExpandProjectedMotion(invalid,std::numeric_limits<double>::max(),&box));EXPECT_EQ(Bytes(box),before);
    contact::Q4IntegralInterval gap;ASSERT_TRUE(EncloseLeadingGap(-.099,-.1,&gap));const auto saved=Bytes(gap);
    EXPECT_GT(gap.lower,0);EXPECT_FALSE(EncloseLeadingGap(-.1,-.1,&gap));EXPECT_EQ(Bytes(gap),saved);
    double upper=-7;ASSERT_TRUE(CheckContactStep(1./64,16,.125,&upper));const auto original=Bytes(upper);
    EXPECT_EQ(CheckContactStep(1./16,16,.125,&upper).status,PenaltyStatus::StepLimit);EXPECT_EQ(Bytes(upper),original);
    EXPECT_EQ(CheckContactStep(1./64,16,.126,&upper).status,PenaltyStatus::InvalidInput);EXPECT_EQ(Bytes(upper),original);
}
} // namespace
} // namespace crash::cases::wall_penalty
