// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace beam18_force_test {
TEST(Beam18Force, AnalyticSolidMaterialCannotEnterTabulatedBeamCaller) {
  const auto ref=Reference();const auto table=Material(ref);
  b::Material analytic;
  namespace solid=tl::material::law44::solid;
  ASSERT_EQ(solid::PrepareAnalytic(table.material,{20e6,10e6,1},analytic),solid::Status::Ok);
  ASSERT_TRUE(solid::detail::ParametersValid(analytic));
  b::point::Result result;result.history.stress_pa[0]=123;
  const auto saved=result;
  EXPECT_EQ(b::point::Update(analytic,{}, {},result),solid::Status::InvalidParameters);
  EXPECT_EQ(std::memcmp(&result,&saved,sizeof(result)),0);
  auto prepared=table;
  EXPECT_EQ(b::point::Prepare(analytic.material,{},prepared),solid::Status::InvalidParameters);
  EXPECT_EQ(prepared.curve.plastic_strain,table.curve.plastic_strain);
  b::ForceTrial trial;
  EXPECT_EQ(b::InitializeForce(ref,analytic,{},trial),b::Status::InvalidInput);
  EXPECT_FALSE(trial.proposed_history.prepared());
  EXPECT_EQ(b::InitializeForce(ref,table,{},trial),b::Status::Success);
}
TEST(Beam18Force, BeamPointProjectionUsesThreeStressMetricAndForwardCurveCursor) {
  const auto material = Material(Reference());
  b::point::History accepted;
  b::point::Input input;
  input.strain_increment[0] = .004;
  input.strain_increment[1] = .003;
  input.strain_increment[2] = -.002;
  input.total_axial_strain = .004;
  b::point::Result output;
  ASSERT_EQ(b::point::Update(material, accepted, input, output), b::point::Status::Ok);
  const double normal = material.material.young_pa * input.strain_increment[0];
  const double shear_y = (5. / 6.) * material.shear_pa * input.strain_increment[1];
  const double shear_z = (5. / 6.) * material.shear_pa * input.strain_increment[2];
  const double equivalent = std::sqrt(normal * normal + 3. * (shear_y * shear_y + shear_z * shear_z));
  const double ratio = material.curve.yield_stress_pa[0] / equivalent;
  EXPECT_TRUE(Near(output.history.stress_pa[0], normal * ratio));
  EXPECT_TRUE(Near(output.history.stress_pa[1], shear_y * ratio));
  EXPECT_TRUE(Near(output.history.stress_pa[2], shear_z * ratio));
  EXPECT_TRUE(Near(output.plastic_increment, equivalent * (1. - ratio) / material.material.young_pa));
  EXPECT_EQ(output.tangent_factor, 1.);

  accepted = {};
  accepted.plastic_strain = material.curve.plastic_strain[12];
  input = {};
  ASSERT_EQ(b::point::Update(material, accepted, input, output), b::point::Status::Ok);
  EXPECT_GT(output.history.curve_cursor, 0u);
  EXPECT_TRUE(Near(output.yield_stress_pa, material.curve.yield_stress_pa[12]));
  EXPECT_EQ(output.history.plastic_strain, accepted.plastic_strain);
  EXPECT_EQ(output.plastic_increment, 0.);
}
TEST(Beam18Force, LatePointOverflowAndSourceIdentityMismatchPreserveOutput) {
  const auto ref = Reference();
  const auto material = Material(ref);
  b::point::Input input;
  b::point::Result result;
  ASSERT_EQ(b::point::Update(material, {}, input, result), b::point::Status::Ok);
  const auto saved = result;
  input.strain_increment[2] = std::numeric_limits<double>::max();
  EXPECT_EQ(b::point::Update(material, {}, input, result), b::point::Status::NonfiniteResult);
  EXPECT_EQ(std::memcmp(&saved, &result, sizeof(result)), 0);
  input = {};
  ASSERT_EQ(b::point::Update(material, {}, input, result), b::point::Status::Ok);
  EXPECT_EQ(std::memcmp(&saved, &result, sizeof(result)), 0);

  b::ForceTrial initial, trial;
  ASSERT_EQ(b::InitializeForce(ref, material, {}, initial), b::Status::Success);
  auto changed = ref.input();
  ++changed.source_element_id;
  b::Reference other;
  ASSERT_EQ(b::InitializeReference(changed, other), b::Status::Success);
  trial = initial;
  const auto before = trial;
  EXPECT_EQ(b::EvaluateForce(other, material, initial.proposed_history,
      Motion(ref, initial.proposed_history), trial), b::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(&before, &trial, sizeof(trial)), 0);
}
TEST(Beam18Force, VirginTranslationHasActualZeroForceAndNoCompletedInterval) {
  const auto ref=Reference();const auto material=Material(ref);b::ForceTrial trial;
  ASSERT_EQ(b::InitializeForce(ref,material,{11.123,-.37,.129},trial),b::Status::Success);
  EXPECT_EQ(trial.proposed_history.stamp().sample_index,0u);
  for (unsigned n=0;n<2;++n) {
    EXPECT_EQ(tl::math::fixed3::Norm(trial.rhs_force_n[n]),0);
    EXPECT_EQ(tl::math::fixed3::Norm(trial.rhs_couple_nm[n]),0);
  }
  EXPECT_GT(trial.diagnostics.translation_stiffness_n_m,0);
  EXPECT_GT(trial.diagnostics.rotation_stiffness_nm,0);
  EXPECT_EQ(trial.proposed_history.values().internal_energy_j[0],0);
}
TEST(Beam18Force, AxialElasticForceAndEnergyUseFourPointsAndCurrentLength) {
  const auto ref=Reference();const auto material=Material(ref);b::ForceTrial initial,next;
  ASSERT_EQ(b::InitializeForce(ref,material,{},initial),b::Status::Success);
  auto input=Motion(ref,initial.proposed_history);input.velocity_midpoint_m_s[1]={.1,0,0};
  ASSERT_EQ(b::EvaluateForce(ref,material,initial.proposed_history,input,next),b::Status::Success);
  const double strain=.1/.1*input.dt_s;
  const double force=material.material.young_pa*strain*ref.section().area;
  EXPECT_TRUE(Near(next.rhs_force_n[0].x,force));EXPECT_TRUE(Near(next.rhs_force_n[1].x,-force));
  EXPECT_TRUE(Near(next.proposed_history.values().internal_energy_j[0],.5*force*strain*.1));
  for (const auto& p:next.point) {EXPECT_EQ(p.history.plastic_strain,0);EXPECT_EQ(p.tangent_factor,1);}
}
TEST(Beam18Force, MixedModesKeepEndpointForceCoupleBalanceAndPhysicalDampingSeparate) {
  const auto ref=Reference();const auto material=Material(ref);b::ForceTrial initial,next;
  ASSERT_EQ(b::InitializeForce(ref,material,{},initial),b::Status::Success);
  auto input=Motion(ref,initial.proposed_history);
  input.velocity_midpoint_m_s[1]={0,.03,-.05};
  input.angular_velocity_midpoint_rad_s[0]={1,2,-3};
  input.angular_velocity_midpoint_rad_s[1]={-2,-1,4};
  ASSERT_EQ(b::EvaluateForce(ref,material,initial.proposed_history,input,next),b::Status::Success);
  using namespace tl::math::fixed3;
  const auto force=Add(next.rhs_force_n[0],next.rhs_force_n[1]);
  const auto couple=Add(Add(next.rhs_couple_nm[0],next.rhs_couple_nm[1]),
      Cross(Subtract(input.position_endpoint_m[1],input.position_endpoint_m[0]),next.rhs_force_n[1]));
  EXPECT_NEAR(Norm(force),0,1e-12);EXPECT_NEAR(Norm(couple),0,1e-10);
  EXPECT_NE(next.diagnostics.damped_section_force_n.y,next.proposed_history.values().section_force_n.y);
  EXPECT_NE(next.proposed_history.values().section_seed.z,0);
}
TEST(Beam18Force, PlasticHistoryFilterLateFailureAndRetryAreAtomic) {
  const auto ref=Reference();const auto material=Material(ref);b::ForceTrial accepted,next;
  ASSERT_EQ(b::InitializeForce(ref,material,{},accepted),b::Status::Success);
  for(unsigned step=0;step<8;++step) {
    auto in=Motion(ref,accepted.proposed_history);in.velocity_midpoint_m_s[1]={step<4?1000.:-500.,50.,-25.};
    ASSERT_EQ(b::EvaluateForce(ref,material,accepted.proposed_history,in,next),b::Status::Success);
    EXPECT_GE(next.proposed_history.values().plastic_work_j,accepted.proposed_history.values().plastic_work_j);
    EXPECT_GT(next.proposed_history.values().filtered_neutral_rate_per_s,0);
    const auto saved=next;
    auto bad=in;bad.angular_velocity_midpoint_rad_s[1].z=std::numeric_limits<double>::max();
    EXPECT_NE(b::EvaluateForce(ref,material,accepted.proposed_history,bad,next),b::Status::Success);
    EXPECT_EQ(std::memcmp(&saved,&next,sizeof(next)),0);
    ASSERT_EQ(b::EvaluateForce(ref,material,accepted.proposed_history,in,next),b::Status::Success);
    EXPECT_EQ(std::memcmp(&saved,&next,sizeof(next)),0);
    bad=in;bad.dt_s=0;
    EXPECT_EQ(b::EvaluateForce(ref,material,accepted.proposed_history,bad,next),b::Status::InvalidInput);
    accepted=next;
  }
}
TEST(Beam18Force, EndpointAssemblyLeavesOrientationNodeAndRejectsDuplicateMapping) {
  const auto ref=Reference();const auto material=Material(ref);b::ForceTrial initial,next;
  ASSERT_EQ(b::InitializeForce(ref,material,{},initial),b::Status::Success);
  auto input=Motion(ref,initial.proposed_history);input.velocity_midpoint_m_s[1]={.1,.2,.3};
  ASSERT_EQ(b::EvaluateForce(ref,material,initial.proposed_history,input,next),b::Status::Success);
  double values[6][3]{};for(auto& row:values)row[2]=123.;
  tl::fea::DeviceNodalForceView view{values[0],values[1],values[2],values[3],values[4],values[5],3};
  const std::size_t ids[2]{0,1};
  ASSERT_EQ(b::AccumulateForces(ids,next,view),tl::fea::NodalForceAssemblyStatus::Success);
  for(const auto& row:values)EXPECT_EQ(row[2],123.);
  double saved[6][3];std::memcpy(saved,values,sizeof(values));
  const std::size_t wrong[2]{0,0};
  EXPECT_EQ(b::AccumulateForces(wrong,next,view),tl::fea::NodalForceAssemblyStatus::InvalidConnectivity);
  EXPECT_EQ(std::memcmp(saved,values,sizeof(values)),0);
}
TEST(Beam18Force, RigidFrameChangeRotatesEndpointLoadsAndPreservesSectionHistory) {
  const auto ref = Reference();
  auto rotated_input = ref.input();
  const auto rotate = [](b::Vec3 v) { return b::Vec3{-v.y, v.x, v.z}; };
  for (auto& position : rotated_input.position) position = rotate(position);
  b::Reference rotated;
  ASSERT_EQ(b::InitializeReference(rotated_input, rotated), b::Status::Success);
  const auto material = Material(ref);
  b::ForceTrial initial, rotated_initial, next, rotated_next;
  ASSERT_EQ(b::InitializeForce(ref, material, {}, initial), b::Status::Success);
  ASSERT_EQ(b::InitializeForce(rotated, material, {}, rotated_initial), b::Status::Success);
  auto input = Motion(ref, initial.proposed_history);
  input.velocity_midpoint_m_s[1] = {.2, .03, -.07};
  input.angular_velocity_midpoint_rad_s[0] = {2., -1., 3.};
  input.angular_velocity_midpoint_rad_s[1] = {-3., 4., 2.};
  auto transformed = input;
  for (unsigned n = 0; n < 2; ++n) {
    transformed.position_endpoint_m[n] = rotate(input.position_endpoint_m[n]);
    transformed.velocity_midpoint_m_s[n] = rotate(input.velocity_midpoint_m_s[n]);
    transformed.angular_velocity_midpoint_rad_s[n] = rotate(input.angular_velocity_midpoint_rad_s[n]);
  }
  ASSERT_EQ(b::EvaluateForce(ref, material, initial.proposed_history, input, next), b::Status::Success);
  ASSERT_EQ(b::EvaluateForce(rotated, material, rotated_initial.proposed_history, transformed, rotated_next), b::Status::Success);
  for (unsigned n = 0; n < 2; ++n) {
    EXPECT_NEAR(tl::math::fixed3::Norm(tl::math::fixed3::Subtract(
        rotate(next.rhs_force_n[n]), rotated_next.rhs_force_n[n])), 0., 1e-10);
    EXPECT_NEAR(tl::math::fixed3::Norm(tl::math::fixed3::Subtract(
        rotate(next.rhs_couple_nm[n]), rotated_next.rhs_couple_nm[n])), 0., 1e-11);
  }
  for (unsigned p = 0; p < 4; ++p)
    for (unsigned c = 0; c < 3; ++c)
      EXPECT_DOUBLE_EQ(next.point[p].history.stress_pa[c], rotated_next.point[p].history.stress_pa[c]);
}
} // namespace beam18_force_test
