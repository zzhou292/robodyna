// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <limits>

namespace beam18_test {
TEST(Beam18Reference, FourSourceSectionPointsAndActualEndpointMassUnits) {
  for (double radius : {4.5,5.9}) {
    const auto input = Input(16,radius);
    beam::Reference output;
    ASSERT_EQ(beam::InitializeReference(input,output),beam::Status::Success);
    EXPECT_TRUE(output.prepared());
    EXPECT_EQ(output.input().source_node_id[2],33u);
    const double area = std::acos(-1.0)*radius*radius;
    EXPECT_DOUBLE_EQ(output.section().area,area);
    EXPECT_EQ(output.section().membrane_damping,0);
    EXPECT_EQ(output.section().flexural_damping,.01);
    const double area_inertia = area*area/48+area*radius*radius/4;
    EXPECT_NEAR(output.section().inertia_y,area_inertia,4e-15*area_inertia);
    EXPECT_NEAR(output.section().inertia_z,area_inertia,4e-15*area_inertia);
    EXPECT_EQ(output.native_mass().endpoint_mass,input.density*16*area*.5);
    EXPECT_EQ(output.endpoint().mass_kg,output.native_mass().endpoint_mass*1000);
    EXPECT_EQ(output.endpoint().native_total_inertia_kg_m2,output.native_mass().endpoint_total_inertia*(1000*.001*.001));
    EXPECT_GT(output.endpoint().translation_stiffness_n_m,output.endpoint().interface_stiffness_n_m);
  }
}
TEST(Beam18Reference, LengthThresholdsRetainDistinctInertiaBranches) {
  beam::Reference short_beam, long_beam, floor_beam;
  ASSERT_EQ(beam::InitializeReference(Input(4),short_beam),beam::Status::Success);
  ASSERT_EQ(beam::InitializeReference(Input(20),long_beam),beam::Status::Success);
  ASSERT_EQ(beam::InitializeReference(Input(.01),floor_beam),beam::Status::Success);
  EXPECT_LT(short_beam.native_mass().facdt,1);
  EXPECT_EQ(short_beam.native_mass().axial_coefficient,1.0/12);
  EXPECT_GT(long_beam.native_mass().facdt,1);
  EXPECT_EQ(long_beam.native_mass().axial_coefficient,1);
  EXPECT_EQ(floor_beam.native_mass().endpoint_total_inertia,floor_beam.native_mass().torsional_floor);
  EXPECT_GT(long_beam.native_mass().endpoint_total_inertia,long_beam.native_mass().torsional_floor);
}
TEST(Beam18Reference, OwnedSourceEvidenceKeepsOrientationSeparateFromEndpointCoefficients) {
  auto input = Input();
  beam::Reference first, second;
  ASSERT_EQ(beam::InitializeReference(input,first),beam::Status::Success);
  input.source_node_id[2] = 999;
  input.position[2] = {101,210,340};
  ASSERT_EQ(beam::InitializeReference(input,second),beam::Status::Success);
  EXPECT_EQ(first.input().source_node_id[2],33u);
  EXPECT_EQ(second.input().source_node_id[2],999u);
  EXPECT_EQ(second.native_mass().endpoint_mass,first.native_mass().endpoint_mass);
  EXPECT_EQ(second.native_mass().endpoint_total_inertia,first.native_mass().endpoint_total_inertia);
  EXPECT_EQ(second.endpoint().rotation_stiffness_nm,first.endpoint().rotation_stiffness_nm);
  EXPECT_NE(second.geometry().orientation_seed.z,first.geometry().orientation_seed.z);
  input.radius = 100;
  EXPECT_EQ(first.input().radius,4.5);
  EXPECT_EQ(second.input().radius,4.5);
}
TEST(Beam18Reference, NativeThirdNodeFallbackAndSignedSourceIdentity) {
  auto input = Input(); beam::Reference output;
  ASSERT_EQ(beam::InitializeReference(input,output),beam::Status::Success);
  EXPECT_EQ(output.geometry().orientation_branch,beam::OrientationBranch::ThirdNode);
  input.position[2] = {101,200,300};
  ASSERT_EQ(beam::InitializeReference(input,output),beam::Status::Success);
  EXPECT_EQ(output.geometry().orientation_branch,beam::OrientationBranch::GlobalY);
  input.source_node_id[2] = 0; input.position[2] = {};
  input.position[1] = {100,216,300};
  ASSERT_EQ(beam::InitializeReference(input,output),beam::Status::Success);
  EXPECT_EQ(output.geometry().orientation_branch,beam::OrientationBranch::GlobalZ);
  input.position[0] = {0,0,0}; input.position[1] = {0,16,0};
  input.source_node_id[2] = input.source_node_id[0]; input.position[2] = {-0.0,0,0};
  const auto before = Bytes(output);
  EXPECT_EQ(beam::InitializeReference(input,output),beam::Status::InvalidInput);
  EXPECT_EQ(Bytes(output),before);
  input.position[2] = input.position[0];
  EXPECT_EQ(beam::InitializeReference(input,output),beam::Status::Success);
}
TEST(Beam18Reference, UnsupportedAndLateNonfiniteInputsLeaveOutputUnchanged) {
  auto input = Input(); beam::Reference output;
  ASSERT_EQ(beam::InitializeReference(input,output),beam::Status::Success);
  const auto before = Bytes(output);
  const auto values = Values(output);
  auto bad = input; bad.release[3] = 1;
  EXPECT_EQ(beam::InitializeReference(bad,output),beam::Status::UnsupportedScope);
  EXPECT_EQ(Bytes(output),before);
  bad = input; bad.position[1] = bad.position[0];
  EXPECT_EQ(beam::InitializeReference(bad,output),beam::Status::DegenerateGeometry);
  EXPECT_EQ(Bytes(output),before);
  bad = input; bad.density = std::numeric_limits<double>::max();
  EXPECT_EQ(beam::InitializeReference(bad,output),beam::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(output),before);
  bad = input; bad.source_node_id[1] = bad.source_node_id[0];
  EXPECT_EQ(beam::InitializeReference(bad,output),beam::Status::InvalidInput);
  EXPECT_EQ(Bytes(output),before);
  ASSERT_EQ(beam::InitializeReference(input,output),beam::Status::Success);
  EXPECT_EQ(Values(output),values);
}
} // namespace beam18_test
