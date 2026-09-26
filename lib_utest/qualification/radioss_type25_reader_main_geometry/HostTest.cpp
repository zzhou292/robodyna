// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
namespace reader_main_geometry_test {
TEST(ReaderMainGeometry, InternalNativeReturnRetainsEveryAreaNormalVolumeAndIdentitySlot) {
  const auto cases=Cases();ASSERT_EQ(cases.size(),184u);
  for(std::size_t i=0;i<cases.size();++i) {
    SCOPED_TRACE(i);
    const auto expected=InternalOracle(cases[i]);auto out=Sentinel();
    ASSERT_EQ(n::EvaluateNativeInternalMainGeometry(cases[i],&out),n::CoefficientStatus::Ok);
    Same(out,expected.value);EXPECT_FALSE(expected.reversed);
    for(unsigned k=0;k<4;++k)EXPECT_EQ(expected.source_corner[k],k==3&&cases[i].layout==n::ShellLayout::Triangle3?2u:k);
  }
}
TEST(ReaderMainGeometry, RawSecondVolumePreservesNativeSignsRepeatsAndReductionBits) {
  for(const auto& input:Cases()) {
    double volume=71.;
    ASSERT_EQ(n::EvaluateNativeEightSlotReaderVolume(input.solid_raw,&volume),n::CoefficientStatus::Ok);
    Exact(volume,VolumeOracle(input.solid_raw));Exact(volume,InternalOracle(input).value.signed_volume);
  }
}
TEST(ReaderMainGeometry, InternalEarlyReturnDoesNotValidateUnconsumedExteriorProjection) {
  auto input=Basic();for(auto& x:input.solid_raw)x={std::numeric_limits<double>::max(),0,0};
  n::NativeInternalMainGeometryResult out;
  ASSERT_EQ(n::EvaluateNativeInternalMainGeometry(input,&out),n::CoefficientStatus::Ok);
  Same(out,InternalOracle(input).value);Exact(out.signed_volume,0.);
  auto legacy=main_geometry_test::Sentinel();
  EXPECT_EQ(n::EvaluateNativeExteriorMainGeometry(input,&legacy),n::CoefficientStatus::NonfiniteResult);
  main_geometry_test::Same(legacy,main_geometry_test::Sentinel());
}
TEST(ReaderMainGeometry, AllConsumedFailuresPreserveOutputsAndRetry) {
  const auto cases=Invalid();
  for(unsigned i=0;i<cases.size();++i) {
    SCOPED_TRACE(i);
    auto out=Sentinel();
    EXPECT_EQ(n::EvaluateNativeInternalMainGeometry(cases[i],&out),main_geometry_test::InvalidStatus(i));
    Same(out,Sentinel());
    ASSERT_EQ(n::EvaluateNativeInternalMainGeometry(Basic(),&out),n::CoefficientStatus::Ok);
    Same(out,InternalOracle(Basic()).value);
  }
  double volume=71.;auto raw=Basic();raw.solid_raw[0].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(n::EvaluateNativeEightSlotReaderVolume(raw.solid_raw,&volume),n::CoefficientStatus::InvalidInput);Exact(volume,71.);
  EXPECT_EQ(n::EvaluateNativeInternalMainGeometry(Basic(),nullptr),n::CoefficientStatus::InvalidInput);
  EXPECT_EQ(n::EvaluateNativeEightSlotReaderVolume(Basic().solid_raw,nullptr),n::CoefficientStatus::InvalidInput);
}
}
