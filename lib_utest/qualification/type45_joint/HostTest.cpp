// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>

namespace type45_test {
TEST(Type45Host, ReleasedDofsAndPreAutomaticStartupAreDistinct) {
  for (const auto kind : {Kind::Spherical,Kind::Revolute,Kind::Cylindrical}) {
    Fixture fixture(kind);
    auto reference=fixture.Prepare();
    History history;
    ASSERT_EQ(History::Initialize(reference,history),Status::Success);
    EXPECT_EQ(reference.startup().maximum_translation_n_m,0);
    EXPECT_EQ(reference.startup().maximum_rotation_nm,0);
    EXPECT_GT(reference.automatic_stiffness().blocked_translation_n_m,0);
    auto step=fixture.Step(history);
    step.angular_velocity_rad_s[1]={.1,0,0};
    if (kind==Kind::Cylindrical) step.position_m[1].x+=1e-5;
    Evaluation result;
    ASSERT_EQ(Evaluate(reference,history,step,result),Status::Success);
    EXPECT_EQ(result.history.values().local_couple_nm.x,0);
    if (kind==Kind::Spherical) Near(result.history.values().local_couple_nm,{},0);
    if (kind==Kind::Cylindrical) EXPECT_EQ(result.history.values().local_force_n.x,0);
    EXPECT_EQ(result.history.stamp().sample_index,1u);
    EXPECT_EQ(history.stamp().sample_index,0u);
  }
  Fixture scalar(Kind::Cylindrical);
  scalar.property.free_stiffness.translation.x=7;
  scalar.property.free_viscosity.translation.x=2;
  const auto reference=scalar.Prepare();
  EXPECT_EQ(reference.startup().maximum_translation_n_m,7);
  History history;
  ASSERT_EQ(History::Initialize(reference,history),Status::Success);
  auto step=scalar.Step(history);
  step.position_m[1].x+=1e-5;
  Evaluation result;
  ASSERT_EQ(Evaluate(reference,history,step,result),Status::Success);
  const double delta=step.position_m[1].x-scalar.geometry.position_m[1].x;
  // A free DOF has explicit K/C but no critical damping term.
  EXPECT_DOUBLE_EQ(result.history.values().local_force_n.x,7*delta+2*(delta/step.dt_s));
}

TEST(Type45Host, AutomaticMainInertiaAndDampingMeanRemainIndependent) {
  Fixture fixture;
  const auto reference=fixture.Prepare();
  Fixture damping_only=fixture;
  damping_only.damping[1].mean_principal_inertia_kg_m2=.4;
  const auto changed=damping_only.Prepare();
  EXPECT_DOUBLE_EQ(reference.automatic_stiffness().blocked_rotation_nm,
                   changed.automatic_stiffness().blocked_rotation_nm);
  EXPECT_FALSE(reference.Matches(changed));
  History h1,h2;
  ASSERT_EQ(History::Initialize(reference,h1),Status::Success);
  ASSERT_EQ(History::Initialize(changed,h2),Status::Success);
  auto step=fixture.Step(h1);
  step.angular_velocity_rad_s[1].y=.1;
  Evaluation a,b;
  ASSERT_EQ(Evaluate(reference,h1,step,a),Status::Success);
  ASSERT_EQ(Evaluate(changed,h2,step,b),Status::Success);
  EXPECT_NE(a.history.values().local_couple_nm.y,b.history.values().local_couple_nm.y);
  Fixture zero=fixture;
  zero.context.main[1].inertia_kg_m2=0;
  const auto z=zero.Prepare();
  EXPECT_EQ(z.automatic_stiffness().blocked_rotation_nm,0);
  Fixture raised=fixture;
  raised.context.main[0].translational_stiffness_n_m=1e12;
  const auto r=raised.Prepare();
  EXPECT_TRUE(r.automatic_stiffness().raised_to_structural_stiffness);
  EXPECT_DOUBLE_EQ(r.automatic_stiffness().blocked_translation_n_m,2e12);
  EXPECT_GT(r.automatic_stiffness().blocked_rotation_nm,reference.automatic_stiffness().blocked_rotation_nm);
}

TEST(Type45Host, OffsetWrenchesBalanceAndCommonRotationAdvancesMeanFrame) {
  Fixture fixture;
  const auto reference=fixture.Prepare();
  History accepted;
  ASSERT_EQ(History::Initialize(reference,accepted),Status::Success);
  auto step=fixture.Step(accepted);
  step.position_m[1].y+=.0001;
  step.angular_velocity_rad_s[1]={.3,.2,-.1};
  Evaluation result;
  ASSERT_EQ(Evaluate(reference,accepted,step,result),Status::Success);
  Near(f::Add(result.endpoint[0].force_n,result.endpoint[1].force_n),{},0);
  auto torque=f::Add(f::Cross(step.position_m[0],result.endpoint[0].force_n),
                     f::Cross(step.position_m[1],result.endpoint[1].force_n));
  torque=f::Add(torque,f::Add(result.endpoint[0].couple_nm,result.endpoint[1].couple_nm));
  Near(torque,{},1e-10);
  auto rotated=fixture.Step(accepted);
  const double angle=.02;
  const Matrix3 turn{{::cos(angle),-::sin(angle),0,::sin(angle),::cos(angle),0,0,0,1}};
  for (unsigned i=0; i<2; ++i) {
    rotated.position_m[i]=f::ToWorld(turn,fixture.geometry.position_m[i]);
    rotated.angular_velocity_rad_s[i]={0,0,angle/rotated.dt_s};
  }
  ASSERT_EQ(Evaluate(reference,accepted,rotated,result),Status::Success);
  Near(result.history.values().local_displacement_m,{},1e-17);
  Near(result.history.values().relative_rotation_rad,{},0);
  for (unsigned i=0; i<9; ++i) EXPECT_NEAR(result.history.values().frame.v[i],turn.v[i],1e-15);
}

TEST(Type45Host, LateFailurePreservesCompleteOutputAndRetryAliases) {
  Fixture fixture;
  auto reference=fixture.Prepare();
  History initial;
  ASSERT_EQ(History::Initialize(reference,initial),Status::Success);
  auto step=fixture.Step(initial);
  step.position_m[1].z+=1e-5;
  Evaluation good;
  ASSERT_EQ(Evaluate(reference,initial,step,good),Status::Success);
  auto output=good;
  auto bad=step;
  bad.position_m[1].z=std::numeric_limits<double>::max()/4;
  EXPECT_EQ(Evaluate(reference,initial,bad,output),Status::NonfiniteResult);
  EXPECT_TRUE(output.history.Matches(reference));
  Same(output,good);
  bad=step;
  bad.sample_index=2;
  EXPECT_EQ(Evaluate(reference,initial,bad,output),Status::StaleInterval);
  Same(output,good);
  Fixture changed=fixture;
  changed.geometry.source_node_id[2]=104;
  EXPECT_EQ(Evaluate(changed.Prepare(),initial,step,output),Status::ReferenceMismatch);
  EXPECT_TRUE(output.history.Matches(reference));
  Same(output,good);
  auto next_step=fixture.Step(output.history);
  next_step.position_m[1].x+=2e-5;
  Evaluation next;
  ASSERT_EQ(Evaluate(reference,good.history,next_step,next),Status::Success);
  ASSERT_EQ(Evaluate(reference,output.history,next_step,output),Status::Success);
  Same(output,next);
}

TEST(Type45Host, PreparedIdentityUnitsDegeneracyAndLateContextRejection) {
  Fixture fixture;
  auto reference=fixture.Prepare();
  const auto saved=reference;
  auto invalid=fixture;
  invalid.context.main[1].role=EndpointRole::RigidMain;
  EXPECT_EQ(Reference::Prepare(invalid.property,invalid.geometry,invalid.damping,
                              invalid.context,reference),Status::UnsupportedRole);
  EXPECT_TRUE(reference.Matches(saved));
  invalid=fixture;
  invalid.geometry.position_m[2]=invalid.geometry.position_m[0];
  EXPECT_EQ(Reference::Prepare(invalid.property,invalid.geometry,invalid.damping,
                              invalid.context,reference),Status::InvalidGeometry);
  EXPECT_TRUE(reference.Matches(saved));
  invalid=fixture;
  invalid.geometry.position_m[0].z=-0.;
  EXPECT_FALSE(reference.Matches(invalid.Prepare()));
  Fixture units=fixture;
  units.geometry.position_m[2]={5e-12,0,0};
  EXPECT_EQ(Reference::Prepare(units.property,units.geometry,units.damping,
                              units.context,reference),Status::InvalidGeometry);
  units.property.working_units=WorkingUnits::MillimetreTonneSecond;
  EXPECT_TRUE(units.Prepare().ready());
  units.property.kind=Kind(255);
  EXPECT_EQ(Reference::Prepare(units.property,units.geometry,units.damping,
                              units.context,reference),Status::InvalidProperty);
}
} // namespace type45_test
