#include "RigidGauge.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::cases::native_scene::rigid_trajectory_test {
namespace {
const gauge::Matrix identity{1,0,0,0,1,0,0,0,1};
// Right-handed signed permutation: native axes3,-2,1. It is not itself
// a physical rotation of the body when its principal moments are permuted.
const gauge::Matrix permutation{0,0,1,0,-1,0,1,0,0};
const gauge::Moments native{1,2,4},permuted{4,2,1};
const gauge::Matrix rotate{0,-1,0,1,0,0,0,0,1};
}
TEST(NativeRigidFrameGauge, SignedPermutationAuthenticatesTheSameWorldInertia) {
    gauge::Alignment initial;
    ASSERT_TRUE(gauge::Bind(permutation,permuted,identity,native,&initial));
    const auto actual=gauge::WorldInertia(permutation,permuted),wanted=gauge::WorldInertia(identity,native);
    EXPECT_EQ(actual,wanted);EXPECT_TRUE(gauge::SameRotation(initial,permutation,identity));
    EXPECT_NE(permutation,identity);
}
TEST(NativeRigidFrameGauge, WrongMomentsOrPhysicalInitialReorientationDoNotPassAsGauge) {
    gauge::Alignment initial{identity,permutation};const auto before=initial;
    EXPECT_FALSE(gauge::Bind(permutation,native,identity,native,&initial));
    EXPECT_EQ(initial.actual_initial,before.actual_initial);EXPECT_EQ(initial.native_initial,before.native_initial);
    auto wrong=permuted;wrong[1]*=1.001;
    EXPECT_FALSE(gauge::Bind(permutation,wrong,identity,native,&initial));
    EXPECT_FALSE(gauge::Bind(rotate,native,identity,native,&initial));
    // A tiny physical tensor must not pass an unscaled fixed absolute budget.
    EXPECT_FALSE(gauge::Bind(identity,{1e-12,2e-12,4e-12},identity,{1e-12,3e-12,4e-12},&initial));
}
TEST(NativeRigidFrameGauge, RelativeRotationPreservesDynamicsAndRejectsDifferentMotion) {
    gauge::Alignment initial;
    ASSERT_TRUE(gauge::Bind(permutation,permuted,identity,native,&initial));
    const auto moved=gauge::Product(rotate,permutation);
    EXPECT_TRUE(gauge::SameRotation(initial,moved,rotate));
    EXPECT_FALSE(gauge::SameRotation(initial,moved,identity));
}
TEST(NativeRigidFrameGauge, NonfiniteNonorthogonalAndImproperFramesReject) {
    gauge::Alignment initial;
    auto reflected=identity;reflected[0]=-1;
    EXPECT_FALSE(gauge::Bind(reflected,native,identity,native,&initial));
    auto distorted=identity;distorted[0]=1.01;
    EXPECT_FALSE(gauge::Bind(distorted,native,identity,native,&initial));
    auto bad=identity;bad[0]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(gauge::Bind(bad,native,identity,native,&initial));
    EXPECT_FALSE(gauge::Bind(identity,{1,0,4},identity,native,&initial));
}
}
