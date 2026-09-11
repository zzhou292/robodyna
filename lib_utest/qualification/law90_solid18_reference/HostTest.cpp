// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <gtest/gtest.h>
#include <type_traits>
using namespace law90_reference_test;
TEST(Law90Solid18Reference, DistinctProfileAndExactAllocation) {
  static_assert(sizeof(t::ReferenceCoefficients)==607*sizeof(double));
  static_assert(std::is_trivially_copyable_v<t::Reference>);
  RecordProperty("reference_bytes",static_cast<int>(sizeof(t::Reference)));
  RecordProperty("reference_scratch_bytes",static_cast<int>(sizeof(t::ReferenceScratch)));
  RecordProperty("current_bytes",static_cast<int>(sizeof(t::Kinematics)));
  RecordProperty("current_scratch_bytes",static_cast<int>(sizeof(t::KinematicsScratch)));
  const auto input=Cube();
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  EXPECT_NEAR(reference.mass().element_mass_kg,772,2e-12);
  s::Reference old;
  EXPECT_EQ(s::InitializeReference(input,old),s::Status::UnsupportedProfile);
  for (int s::ResolvedProfile::* field : {&s::ResolvedProfile::material_law,
      &s::ResolvedProfile::pressure,&s::ResolvedProfile::small_strain,
      &s::ResolvedProfile::integration,&s::ResolvedProfile::convected_frame}) {
    auto bad=input;
    bad.profile.*field += 1;
    const auto before=Bytes(reference);
    EXPECT_EQ(t::InitializeReference90(bad,reference),s::Status::UnsupportedProfile);
    EXPECT_EQ(Bytes(reference),before);
  }
}
TEST(Law90Solid18Reference, NativeSlotCorrectionKeepsEarlierSourceSnapshot) {
  auto input=Distorted();
  for(unsigned n=0;n<4;++n) {
    std::swap(input.position_m[n],input.position_m[n+4]);
    std::swap(input.source_node_id[n],input.source_node_id[n+4]);
  }
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  for(unsigned n=0;n<8;++n) EXPECT_EQ(reference.source_slot(n),(n+4)%8);
  for(unsigned n=0;n<7;++n) {
    const auto& saved=reference.coefficients().source_relative_position_m[n];
    EXPECT_EQ(saved.x,input.position_m[n].x-input.position_m[7].x);
    EXPECT_EQ(saved.y,input.position_m[n].y-input.position_m[7].y);
    EXPECT_EQ(saved.z,input.position_m[n].z-input.position_m[7].z);
  }
  for(double m:reference.mass().source_nodal_mass_kg)
    EXPECT_EQ(m,reference.mass().source_nodal_mass_kg[0]);
  EXPECT_NE(reference.geometry().point[0].initial_volume_m3,
            reference.geometry().point[7].initial_volume_m3);
  const auto values=ReferenceValues(reference);
  ASSERT_EQ(t::InitializeReference90(reference.input(),reference),s::Status::Success);
  EXPECT_EQ(ReferenceValues(reference),values);
}
TEST(Law90Solid18Reference, AffineTotalTensorAndIndependentQuadraticRate) {
  const auto input=Cube();
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  auto current=Current(input);
  current.dt_s=.02;
  for(unsigned n=0;n<8;++n) {
    const auto x=input.position_m[n];
    current.position_m[n]={1.1*x.x, .8*x.y, 1.2*x.z};
    current.velocity_m_s[n]={.4*current.position_m[n].x,
                            -.3*current.position_m[n].y,.2*current.position_m[n].z};
  }
  t::Kinematics value;
  ASSERT_EQ(t::EvaluateKinematics90(reference,current,value),s::Status::Success);
  // Native SRCOOR3/SORTHO3 cube axes are world Y,Z,X, not identity.
  const double expected_frame[9]={0,0,1,1,0,0,0,1,0};
  for(unsigned k=0;k<9;++k) EXPECT_NEAR(value.geometry.frame.v[k],expected_frame[k],1e-15);
  for(const auto& p:value.point) {
    const auto& b=p.selected_left_cauchy_green_minus_identity;
    EXPECT_NEAR(b[0],-.36,2e-14);
    EXPECT_NEAR(b[1],.44,2e-14);
    EXPECT_NEAR(b[2],.21,2e-14);
    for(unsigned k=3;k<6;++k) EXPECT_NEAR(b[k],0,2e-14);
    EXPECT_NEAR(p.engineering_rate_per_s[0],-.3-.01*.09,2e-14);
    EXPECT_NEAR(p.engineering_rate_per_s[1],.2-.01*.04,2e-14);
    EXPECT_NEAR(p.engineering_rate_per_s[2],.4-.01*.16,2e-14);
  }
  EXPECT_EQ(CurrentValues(value).size(),CurrentCount);
}
TEST(Law90Solid18Reference, SeparateShearPlanesAndDiagonalValues) {
  t::PointKinematics point[8];
  for(unsigned ip=0;ip<8;++ip) {
    auto& g=point[ip].material_displacement_gradient;
    g[1]=.01*(ip+1);
    g[2]=.02*(ip+1);
    g[5]=-.03*(ip+1);
  }
  ASSERT_EQ(t::detail::SelectedTensor(point),s::Status::Success);
  for(unsigned ip=0;ip<8;++ip) {
    const auto& b=point[ip].selected_left_cauchy_green_minus_identity;
    EXPECT_DOUBLE_EQ(b[3],point[ip^2].selected_left_cauchy_green_minus_identity[3]);
    EXPECT_DOUBLE_EQ(b[5],point[ip^1].selected_left_cauchy_green_minus_identity[5]);
    EXPECT_DOUBLE_EQ(b[4],point[ip^4].selected_left_cauchy_green_minus_identity[4]);
  }
  EXPECT_NE(point[0].selected_left_cauchy_green_minus_identity[3],
            point[1].selected_left_cauchy_green_minus_identity[3]);
  EXPECT_NE(point[0].selected_left_cauchy_green_minus_identity[0],
            point[1].selected_left_cauchy_green_minus_identity[0]);
}
TEST(Law90Solid18Reference, RejectedLateValuesAndExplicitScratchRetry) {
  const auto input=Distorted();
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  const auto reference_before=Bytes(reference);
  auto bad=input;
  bad.source_node_id[7]=bad.source_node_id[0];
  EXPECT_EQ(t::InitializeReference90(bad,reference),s::Status::InvalidInput);
  EXPECT_EQ(Bytes(reference),reference_before);
  auto current=Path(input,.7);
  t::Kinematics output;
  ASSERT_EQ(t::EvaluateKinematics90(reference,current,output),s::Status::Success);
  const auto before=Bytes(output);
  auto invalid=current;
  invalid.velocity_m_s[7].z=std::numeric_limits<double>::max();
  EXPECT_EQ(t::EvaluateKinematics90(reference,invalid,output),s::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(output),before);
  invalid=current;
  invalid.position_m[6]={.1,.1,-1};
  EXPECT_NE(t::EvaluateKinematics90(reference,invalid,output),s::Status::Success);
  EXPECT_EQ(Bytes(output),before);
  t::KinematicsScratch scratch;
  ASSERT_EQ(t::EvaluateKinematics90Scratch(reference,current,scratch),s::Status::Success);
  EXPECT_EQ(CurrentValues(scratch.staged),CurrentValues(output));
  EXPECT_EQ(Bytes(reference),reference_before);
}

#include "WorkingModeBound.h"
TEST(Law90Solid18Reference, WorkingModeForwardBoundUsesOperandScale) {
  constexpr int signs[4][8]{{1,1,-1,-1,-1,-1,1,1},
      {1,-1,-1,1,-1,1,1,-1},{1,-1,1,-1,1,-1,1,-1},
      {-1,1,-1,1,1,-1,1,-1}};
  for(unsigned sample=0;sample<32;++sample) {
    std::array<double,ReferenceCount> a{},b{};
    double working[8][3]{};
    for(unsigned n=0;n<8;++n)for(unsigned axis=0;axis<3;++axis) {
      working[n][axis]=937.25+sample*.07+n*.0037+axis*.0189;
      b[9+3*n+axis]=working[n][axis]*.001;
      a[9+3*n+axis]=std::nextafter(b[9+3*n+axis],n%2 ? 0.0 : 2.0);
    }
    for(unsigned mode=0;mode<4;++mode)for(unsigned axis=0;axis<3;++axis) {
      double si=signs[mode][0]*a[9+axis],mm=signs[mode][0]*working[0][axis];
      for(unsigned n=1;n<8;++n) {
        si+=signs[mode][n]*a[9+3*n+axis];
        mm+=signs[mode][n]*working[n][axis];
      }
      const unsigned index=42+3*mode+axis;
      a[index]=si;b[index]=mm*.001;
      const auto bound=HigherModeWorkingBound(a,b,mode,axis);
      EXPECT_LE(std::abs(static_cast<long double>(a[index])-b[index]),bound);
      auto bad=b;bad[index]+=1e-10;
      EXPECT_GT(std::abs(static_cast<long double>(a[index])-bad[index]),
                HigherModeWorkingBound(a,bad,mode,axis));
    }
  }
}
