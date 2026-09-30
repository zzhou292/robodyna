// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"

namespace type45_test {
TEST(Type45Native, AllThreeKindsBothUnitsAndScalarFreeCoefficients) {
  for (auto kind : {Kind::Spherical,Kind::Revolute,Kind::Cylindrical}) {
    for (auto units : {WorkingUnits::SI,WorkingUnits::MillimetreTonneSecond}) {
      SCOPED_TRACE(static_cast<int>(kind));
      SCOPED_TRACE(static_cast<int>(units));
      Fixture fixture(kind);
      fixture.property.working_units=units;
      for (unsigned i=0; i<6; ++i) {
        if (!tl::fea::type45::detail::Blocked(kind,i)) {
          tl::fea::type45::detail::Set(fixture.property.free_stiffness,i,3.+i);
          tl::fea::type45::detail::Set(fixture.property.free_viscosity,i,.02*(i+1));
        }
      }
      int status=-1;
      NativeOracle native(fixture,status);
      ASSERT_EQ(status,0);
      const auto reference=fixture.Prepare();
      CompareReference(fixture,reference,native);
      History accepted;
      ASSERT_EQ(History::Initialize(reference,accepted),Status::Success);
      for (unsigned i=0; i<32; ++i) {
        SCOPED_TRACE(i);
        auto step=fixture.Step(accepted);
        step.position_m[1].x+=2e-5*::sin(.2*(i+1));
        step.position_m[1].y-=3e-5*::cos(.17*(i+1));
        step.angular_velocity_rad_s[0]={.13,-.07,.03};
        step.angular_velocity_rad_s[1]={.23,.19,-.11};
        Evaluation output;
        ASSERT_EQ(Evaluate(reference,accepted,step,output),Status::Success);
        type45_native::Step expected;
        ASSERT_TRUE(native.Step(step,expected));
        CompareStep(output,native,expected);
        accepted=output.history;
      }
    }
  }
}

TEST(Type45Native, MainNodeZeroInertiaRaisedStiffnessAndStructuralRegistration) {
  for (unsigned mode=0; mode<4; ++mode) {
    Fixture fixture(Kind::Cylindrical);
    if (mode==0) fixture.context.main[1].inertia_kg_m2=0;
    if (mode==1) fixture.context.main[0].translational_stiffness_n_m=1e12;
    if (mode>=2) {
      for (unsigned i=0; i<2; ++i) {
        auto& c=fixture.context.main[i];
        c.role=EndpointRole::Structural;
        c.source_body_id=0;
        c.position_m=fixture.geometry.position_m[i];
        fixture.damping[i]={c.mass_kg,c.inertia_kg_m2};
      }
    }
    if (mode==3) fixture.context.main[0].inertia_kg_m2=fixture.damping[0].mean_principal_inertia_kg_m2=0;
    int status=-1;
    NativeOracle native(fixture,status);
    ASSERT_EQ(status,0);
    const auto reference=fixture.Prepare();
    CompareReference(fixture,reference,native);
    History accepted;
    ASSERT_EQ(History::Initialize(reference,accepted),Status::Success);
    auto step=fixture.Step(accepted);
    step.position_m[1].y+=3e-7;
    step.angular_velocity_rad_s[1]={.03,.07,.09};
    Evaluation output;
    ASSERT_EQ(Evaluate(reference,accepted,step,output),Status::Success);
    type45_native::Step expected;
    ASSERT_TRUE(native.Step(step,expected));
    CompareStep(output,native,expected);
  }
  Fixture main_node;
  main_node.context.main[1].role=EndpointRole::RigidMain;
  int status=-1;
  NativeOracle rejected(main_node,status);
  EXPECT_EQ(status,2);
}

TEST(Type45Native, NativeAxisFloorAndLargestCoordinateTie) {
  for (auto units : {WorkingUnits::SI,WorkingUnits::MillimetreTonneSecond}) {
    Fixture fixture;
    fixture.property.working_units=units;
    for (Vec3 axis : {Vec3{.25,.25,.25},Vec3{.25,.25,0},Vec3{0,0,.25}}) {
      fixture.geometry.position_m[2]=axis;
      int status=-1;
      NativeOracle native(fixture,status);
      ASSERT_EQ(status,0);
      CompareReference(fixture,fixture.Prepare(),native);
    }
    const double length=units==WorkingUnits::SI ? 1 : .001;
    fixture.geometry.position_m[2]={.5e-10*length,0,0};
    int status=-1;
    NativeOracle rejected(fixture,status);
    EXPECT_EQ(status,2);
  }
}

TEST(Type45Native, WorkingUnitMassAndInertiaStiffnessFloorsKeepNativeBranches) {
  for(auto units:{WorkingUnits::SI,WorkingUnits::MillimetreTonneSecond}) {
    for(double native_mass:{1.5e-15,4e-15}) {
      Fixture fixture;
      fixture.property.working_units=units;
      const double mass_scale=units==WorkingUnits::SI ? 1 : 1000;
      const double length_scale=units==WorkingUnits::SI ? 1 : .001;
      const double inertia_scale=mass_scale*length_scale*length_scale;
      fixture.geometry.position_m[1]=fixture.geometry.position_m[0];
      for(unsigned i=0;i<2;++i) {
        fixture.context.main[i]={EndpointRole::Structural,0,fixture.geometry.position_m[i],
          native_mass*mass_scale,native_mass*inertia_scale,0,0};
        fixture.damping[i]={native_mass*mass_scale,native_mass*inertia_scale};
      }
      const auto reference=fixture.Prepare();
      History history;
      ASSERT_EQ(History::Initialize(reference,history),Status::Success);
      auto step=fixture.Step(history);
      Evaluation actual;
      ASSERT_EQ(Evaluate(reference,history,step,actual),Status::Success);
      int status=-1;
      NativeOracle native(fixture,status);
      ASSERT_EQ(status,0);
      type45_native::Step expected;
      ASSERT_TRUE(native.Step(step,expected));
      const double kt=expected.values[6]*mass_scale;
      const double kr=expected.values[7]*inertia_scale;
      EXPECT_NEAR(actual.endpoint[0].translational_stiffness_n_m,kt,2e-12*kt);
      EXPECT_NEAR(actual.endpoint[0].rotational_stiffness_nm,kr,2e-12*kr);
      if(native_mass==1.5e-15) {
        EXPECT_DOUBLE_EQ(actual.endpoint[0].translational_stiffness_n_m,
                         2*actual.diagnostics.maximum_stiffness_n_m);
        EXPECT_DOUBLE_EQ(actual.endpoint[0].rotational_stiffness_nm,
                         actual.diagnostics.maximum_rotational_stiffness_nm);
      } else {
        EXPECT_GT(actual.endpoint[0].translational_stiffness_n_m,
                  2*actual.diagnostics.maximum_stiffness_n_m);
        EXPECT_GT(actual.endpoint[0].rotational_stiffness_nm,
                  actual.diagnostics.maximum_rotational_stiffness_nm);
      }
    }
  }
}
} // namespace type45_test
