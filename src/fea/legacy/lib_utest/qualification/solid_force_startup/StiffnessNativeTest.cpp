// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../solid24_force/TestSupport.h"
#include "../solid6z_force/TestSupport.h"
#include "lib_src/elements/solids/ForceStiffness.h"
#include <gtest/gtest.h>
extern "C" void solid_startup_native_stiffness(int,double,double*,int*);
TEST(SolidStartupNative, ExactNativeScalingUsesActualSixAndEightSlots) {
  namespace a=tl::fea::solid24;
  namespace b=tl::fea::solid6z;
  a::ForceTrial brick;
  const auto reference=heph_test::Reference();
  ASSERT_EQ(a::InitializeForce(reference,heph_test::Material(reference.input().density_kg_m3),{},brick),
            a::ForceStatus::Success);
  b::ForceTrial wedge;
  ASSERT_EQ(b::InitializeForce(solid6z_force_test::Reference(),solid6z_force_test::Material(),{},{},wedge),
            b::Status::Success);
  for (double raw : {1.0,17.234567891,1e-200,1e200}) {
    // Independent exact extract distinguishes multiplication by native THIRD
    // from an accidental /8, one-half, or doubled nodal mass reconstruction.
    brick.diagnostics.material.raw_stiffness_n_m=raw;
    wedge.material.raw_stiffness_n_m=raw;
    double expected=0;int status=-1;
    tl::fea::solids::NodalStiffness actual;
    solid_startup_native_stiffness(8,raw,&expected,&status);
    ASSERT_EQ(status,0);ASSERT_TRUE(tl::fea::solids::PrepareNodalStiffness(brick,actual));
    EXPECT_EQ(actual.translation_n_m,expected);EXPECT_EQ(actual.rotation_nm,0);
    solid_startup_native_stiffness(6,raw,&expected,&status);
    ASSERT_EQ(status,0);ASSERT_TRUE(tl::fea::solids::PrepareNodalStiffness(wedge,actual));
    EXPECT_EQ(actual.translation_n_m,expected);EXPECT_EQ(actual.rotation_nm,0);
    EXPECT_NE(actual.translation_n_m,raw/8);
  }
}
