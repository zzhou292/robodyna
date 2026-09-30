// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "../radioss_type25_coefficients/NativeOracle.h"
#include "../radioss_type25_coefficients/Assertions.h"
namespace type25_contribution_test {
TEST(Type25NodalContributions, CompleteSolidDefinedSlotsMatchOriginalSourceBits) {
  const auto inputs=SolidCases();
  for(std::size_t i=0;i<inputs.size();++i) {
    SCOPED_TRACE(i);
    n::NativeSolidNodalShares value;
    ASSERT_EQ(n::EvaluateNativeSolidNodalShares(inputs[i],&value),n::CoefficientStatus::Ok);
    CompareSolid(inputs[i],value);
  }
}
TEST(Type25NodalContributions, PentaRawSlotsAndRepeatedHexOccurrencesStayDistinct) {
  n::NativeSolidNodalInput in{n::SolidNodalKind::Penta6,6,1,5};
  n::NativeSolidNodalShares value;
  ASSERT_EQ(n::EvaluateNativeSolidNodalShares(in,&value),n::CoefficientStatus::Ok);
  EXPECT_EQ(value.defined_raw_slot_mask,0x77u);
  const unsigned penta_nodes[8]{0,1,2,2,3,4,5,5};
  double nodal[6]{};
  for(unsigned slot=0;slot<8;++slot)if(value.defined_raw_slot_mask&(1u<<slot))nodal[penta_nodes[slot]]+=value.volume_share;
  for(double x:nodal)Exact(x,1.);
  in={n::SolidNodalKind::Hex8,8,1,5};
  ASSERT_EQ(n::EvaluateNativeSolidNodalShares(in,&value),n::CoefficientStatus::Ok);
  const unsigned hex_nodes[8]{0,0,1,2,3,4,5,5};
  for(auto& x:nodal)x=0;
  for(unsigned slot=0;slot<8;++slot)if(value.defined_raw_slot_mask&(1u<<slot))nodal[hex_nodes[slot]]+=value.volume_share;
  Exact(nodal[0],2.);Exact(nodal[5],2.);
  // A fixture occurrence sum only; no production SPMD ordering/authority claim.
}
TEST(Type25NodalContributions, FilledBulkAssociationAndOverflowRemainVisible) {
  n::NativeSolidNodalInput in{n::SolidNodalKind::Hex8,1e-300,1e300,1e-300};
  n::NativeSolidNodalShares value;
  ASSERT_EQ(n::EvaluateNativeSolidNodalShares(in,&value),n::CoefficientStatus::Ok);
  CompareSolid(in,value);
  EXPECT_GT(value.bulk_volume_share,0.);
  const double wrong=in.fill*(in.bulk*value.volume_share);
  EXPECT_EQ(wrong,0.);
  const n::NativeSolidNodalShares saved{73.,-91.,0x35};value=saved;
  in={n::SolidNodalKind::Hex8,0,std::numeric_limits<double>::max(),2};
  EXPECT_EQ(n::EvaluateNativeSolidNodalShares(in,&value),n::CoefficientStatus::NonfiniteResult);
  Same(value,saved);
  const auto native=OracleSolid(in,-873.25);
  EXPECT_TRUE(std::isnan(native.bulk_volume[0]));
  in={n::SolidNodalKind::Hex8,8,1,5};
  ASSERT_EQ(n::EvaluateNativeSolidNodalShares(in,&value),n::CoefficientStatus::Ok);
  CompareSolid(in,value);
}
TEST(Type25NodalContributions, SpringProductsMaximaZeroSignsAndFloorsMatchNativeBits) {
  const auto inputs=SpringCases();
  for(std::size_t i=0;i<inputs.size();++i) {
    SCOPED_TRACE(i);
    n::NativeScalarCoefficient value;
    ASSERT_EQ(n::EvaluateNativeSpringNodalCoefficient(inputs[i],&value),n::CoefficientStatus::Ok);
    Exact(value.value,ReferenceSpring(inputs[i]));
  }
}
TEST(Type25NodalContributions, ActualNativeLengthPreparationAndItsDiagnosticsStaySeparate) {
  for(auto kind:{n::SpringNodalKind::Type13,n::SpringNodalKind::Type25}) {
    for(int mode:{-2,0,1,3}) {
      SCOPED_TRACE(mode);
      std::array<double,6> positions{0,0,0,.3,.4,0};
      if(mode<=0)positions.fill(std::numeric_limits<double>::quiet_NaN());
      const auto native=OracleLength(kind,mode,positions);
      EXPECT_EQ(native.diagnostics,0);
      auto in=Spring(kind);in.length_mode=mode;in.geometric_length=mode>0?native.value:std::numeric_limits<double>::quiet_NaN();
      n::NativeScalarCoefficient value;
      ASSERT_EQ(n::EvaluateNativeSpringNodalCoefficient(in,&value),n::CoefficientStatus::Ok);
      Exact(value.value,OracleSpringPrepared(in,native.value));
      if(mode<=0)Exact(native.value,1.);
    }
    const auto invalid_full_geometry=OracleLength(kind,1,{0,0,0,0,0,0});
    Exact(invalid_full_geometry.value,0.);
    EXPECT_EQ(invalid_full_geometry.diagnostics,1);
    // The leaf's prepared-XL floor coupons do not claim this whole source
    // geometry would be accepted by RINIT3's earlier NOISE check.
  }
}
TEST(Type25NodalContributions, InactiveAndUnusedSpringOperandsDoNotPublishOrRequireValues) {
  auto in=Spring(n::SpringNodalKind::Type25);
  in.translation[2]={std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()};
  in.length_mode=0;in.geometric_length=std::numeric_limits<double>::quiet_NaN();
  n::NativeScalarCoefficient out{71.};
  ASSERT_EQ(n::EvaluateNativeSpringNodalCoefficient(in,&out),n::CoefficientStatus::Ok);
  Exact(out.value,15.);Exact(out.value,ReferenceSpring(in));
  in.interface_initialization=0;
  for(auto& x:in.translation)x={std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()};
  out.value=71.;
  EXPECT_EQ(n::EvaluateNativeSpringNodalCoefficient(in,&out),n::CoefficientStatus::UnsupportedProfile);
  Exact(out.value,71.);Exact(OracleSpringPrepared(in,std::numeric_limits<double>::quiet_NaN(),71.),71.);
  in=Spring();in.translation[0].slope=std::numeric_limits<double>::max();in.translation[0].scale=2;
  EXPECT_EQ(n::EvaluateNativeSpringNodalCoefficient(in,&out),n::CoefficientStatus::NonfiniteResult);
  Exact(out.value,71.);
  in=Spring();in.geometric_length=-1;
  EXPECT_EQ(n::EvaluateNativeSpringNodalCoefficient(in,&out),n::CoefficientStatus::InvalidInput);
  Exact(out.value,71.);
  in=Spring();
  ASSERT_EQ(n::EvaluateNativeSpringNodalCoefficient(in,&out),n::CoefficientStatus::Ok);
  Exact(out.value,ReferenceSpring(in));
}
TEST(Type25NodalContributions, UnsupportedKindsAndNonfiniteInputsPreserveTypedOutput) {
  const n::NativeSolidNodalShares before{13.,19.,0x21};auto out=before;
  n::NativeSolidNodalInput in;
  EXPECT_EQ(n::EvaluateNativeSolidNodalShares(in,&out),n::CoefficientStatus::UnsupportedProfile);
  Same(out,before);
  in={n::SolidNodalKind::Hex8,std::numeric_limits<double>::quiet_NaN(),1,5};
  EXPECT_EQ(n::EvaluateNativeSolidNodalShares(in,&out),n::CoefficientStatus::InvalidInput);
  Same(out,before);
  EXPECT_EQ(n::EvaluateNativeSolidNodalShares(in,nullptr),n::CoefficientStatus::InvalidInput);
  auto spring=Spring();spring.kind=static_cast<n::SpringNodalKind>(27);n::NativeScalarCoefficient scalar{17.};
  EXPECT_EQ(n::EvaluateNativeSpringNodalCoefficient(spring,&scalar),n::CoefficientStatus::UnsupportedProfile);
  Exact(scalar.value,17.);
  EXPECT_EQ(n::EvaluateNativeSpringNodalCoefficient(Spring(),nullptr),n::CoefficientStatus::InvalidInput);
}
TEST(Type25NodalContributions, GenuineSharesAndSpringStrFeedExistingAsstifiOnce) {
  const n::NativeSolidNodalInput hex{n::SolidNodalKind::Hex8,8,.5,200},penta{n::SolidNodalKind::Penta6,6,1,300};
  const auto spring=Spring();n::NativeSolidNodalShares a,b;n::NativeScalarCoefficient k;
  ASSERT_EQ(n::EvaluateNativeSolidNodalShares(hex,&a),n::CoefficientStatus::Ok);
  ASSERT_EQ(n::EvaluateNativeSolidNodalShares(penta,&b),n::CoefficientStatus::Ok);
  ASSERT_EQ(n::EvaluateNativeSpringNodalCoefficient(spring,&k),n::CoefficientStatus::Ok);
  const auto na=OracleSolid(hex,-873.25),nb=OracleSolid(penta,-873.25);
  n::NativeAccumulatedNodalCoefficients input{a.volume_share+b.volume_share,a.bulk_volume_share+b.bulk_volume_share,17,2,19+k.value};
  n::NativeAccumulatedNodalCoefficients expected_input{na.volume[0]+nb.volume[0],na.bulk_volume[0]+nb.bulk_volume[0],17,2,19+ReferenceSpring(spring)};
  n::NativeNodalCoefficientResult value;
  ASSERT_EQ(n::FinalizeNativeNodalCoefficient(input,&value),n::CoefficientStatus::Ok);
  const auto expected=type25_coefficient_test::Oracle(expected_input);
  type25_coefficient_test::Number(value.normalized_bulk,expected.normalized_bulk,true);
  type25_coefficient_test::Number(value.stiffness,expected.stiffness);
  //19 is an explicitly supplied prior scalar in this value coupon; no beam/mass/STI rule is inferred.
}
} // namespace type25_contribution_test
