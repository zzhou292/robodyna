// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "Assertions.h"
#include "NativeOracle.h"
#include <limits>
#include <stdexcept>
namespace type25_geometry_test {
TEST(Type25Geometry, CompleteSelectedBranchCorpusMatchesIndependentNativeGeometry) {
  const auto cases=Cases(); ASSERT_EQ(cases.size(),139u);
  for (const auto& c:cases) {
    SCOPED_TRACE(c.name); n::NativeRawGeometryResult actual;
    ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),c.input,&actual),n::GeometryStatus::Ok);
    Same(actual,OracleRaw(Profile(),c.input));
  }
}
TEST(Type25Geometry, CenterSubtriangleAndTriangleWeightsRemainNative) {
  for (unsigned sector=0;sector<4;++sector) {
    auto in=Quad(sector);AtBarycentric(in,.2);n::NativeRawGeometryResult out;
    ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),in,&out),n::GeometryStatus::Ok);
    for (unsigned i=0;i<4;++i)
      EXPECT_DOUBLE_EQ(out.weights[i],.125+((i==sector||i==(sector+1)%4)? .25:0));
  }
  auto in=Triangle();n::NativeRawGeometryResult out;
  ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),in,&out),n::GeometryStatus::Ok);
  EXPECT_DOUBLE_EQ(out.weights[0],.25);EXPECT_DOUBLE_EQ(out.weights[1],.25);
  EXPECT_DOUBLE_EQ(out.weights[2],.5);EXPECT_DOUBLE_EQ(out.weights[3],0);
}
TEST(Type25Geometry, Float32BisectorAdditionRoundsBeforeDoublePromotion) {
  const auto cases=Cases();const auto& in=cases[cases.size()-3].input;
  ASSERT_EQ(cases[cases.size()-3].name,"float32-bisector-sum-rounding");
  const float sum=in.vertex_bisector[0][0].x+in.vertex_bisector[0][1].x;
  ASSERT_NE(double(sum),double(in.vertex_bisector[0][0].x)+double(in.vertex_bisector[0][1].x));
  n::NativeRawGeometryResult out;
  ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),in,&out),n::GeometryStatus::Ok);
  Same(out,OracleRaw(Profile(),in));
  EXPECT_DOUBLE_EQ(out.normal.x,out.normal.y);
}
TEST(Type25Geometry, NonfiniteMalformedAndUnsupportedInputsPreserveCompleteOutput) {
  const auto good=Quad();const auto prior=Sentinel<n::NativeUnitsTag>();
  for (unsigned kind=0;kind<8;++kind) {
    auto in=good;auto profile=Profile();auto actual=prior;
    if(kind==0)in.secondary.x=std::numeric_limits<double>::quiet_NaN();
    if(kind==1)in.selection_code=5;
    if(kind==2)in.main_gap[3]=-1;
    if(kind==3)in.corner_normal[1].x=std::numeric_limits<float>::infinity();
    if(kind==4)profile.sharp=2;
    if(kind==5)profile.foreign_row=true;
    if(kind==6)profile.adhesion=true;
    if(kind==7)in.main_node_ids[1]=in.main_node_ids[0];
    EXPECT_EQ(n::EvaluateNativeRawGeometry(profile,in,&actual),
        (kind<4||kind==7)?n::GeometryStatus::InvalidInput:n::GeometryStatus::UnsupportedProfile);
    Same(actual,prior,true);
  }
  EXPECT_EQ(n::EvaluateNativeRawGeometry(Profile(),good,nullptr),n::GeometryStatus::InvalidInput);
}
TEST(Type25Geometry, RepeatedEvaluationOverwritesEveryDefinedFieldDeterministically) {
  auto input=Quad();n::NativeRawGeometryResult actual;
  ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),input,&actual),n::GeometryStatus::Ok);
  // A second call is deterministic and overwrites every defined output field.
  auto reference=actual;actual=Sentinel<n::NativeUnitsTag>();
  ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),input,&actual),n::GeometryStatus::Ok);
  Same(actual,reference,true);
}

TEST(Type25Geometry, EqualBoundaryReferencesRequireIdenticalStoredBits) {
  auto in=Cases().at(136).input;
  in.boundary_ids[2]=in.boundary_ids[0];
  in.vertex_bisector[2][0]=in.vertex_bisector[0][0];
  in.vertex_bisector[2][1]=in.vertex_bisector[0][1];
  n::NativeRawGeometryResult actual;
  ASSERT_EQ(n::EvaluateNativeRawGeometry(Profile(),in,&actual),n::GeometryStatus::Ok);
  Same(actual,OracleRaw(Profile(),in));
  const auto prior=Sentinel<n::NativeUnitsTag>();actual=prior;
  in.vertex_bisector[2][0].z=-0.0;
  EXPECT_EQ(n::EvaluateNativeRawGeometry(Profile(),in,&actual),n::GeometryStatus::InvalidInput);
  Same(actual,prior,true);
  EXPECT_ANY_THROW(OracleRaw(Profile(),in));
}


TEST(Type25Geometry, UndefinedNativeProjectionIsRejectedWithoutInventingAResult) {
  auto in=Quad();
  in.lb=.9;in.lc=.1;in.neighbors[0]=0;
  in.main_gap[0]=in.main_gap[1]=0;
  in.main_gap[2]=in.main_gap[3]=4;
  in.secondary={.4,-.1,-0x1p-56};
  // Deliberately inconsistent prepared weights; upstream native reachability
  // is unproven. This witnesses an undefined scratch domain, not a result.
  ASSERT_LT(1.-in.lb-in.lc,0.);
  const auto prior=Sentinel<n::NativeUnitsTag>();auto actual=prior;
  EXPECT_EQ(n::EvaluateNativeRawGeometry(Profile(),in,&actual),n::GeometryStatus::InvalidInput);
  Same(actual,prior,true);
  EXPECT_THROW(OracleRaw(Profile(),in),std::invalid_argument);
}

} // namespace type25_geometry_test
