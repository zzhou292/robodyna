// SPDX-License-Identifier: AGPL-3.0-or-later
#include "UnitFixture.h"
#include "Assertions.h"
#include "NativeOracle.h"
namespace type25_geometry_test {
TEST(Type25GeometryUnits, NativeThresholdsAndFloatNormalsSurviveExplicitUnitBoundary) {
  const auto cases=Cases();
  for(auto units:{n::UnitScale{1,1,1},n::UnitScale{.125,8,.5},n::UnitScale{.001,1000,1}}) {
    for(std::size_t index:{0u,4u,91u,136u,137u}) {
      SCOPED_TRACE(index);const auto& in=cases.at(index).input;
      n::SiRawGeometryResult out;
      ASSERT_EQ(n::EvaluateSiRawGeometry(Profile(),units,Si(in,units),&out),n::GeometryStatus::Ok);
      Same(out,Si(OracleRaw(Profile(),in),units));
    }
  }
}
TEST(Type25GeometryUnits, InvalidAndOverflowingConversionsNeverChangePriorOutput) {
  const auto in=Si(Quad(),{1,1,1});const auto prior=Sentinel<n::SiUnitsTag>();
  for(auto units:{n::UnitScale{},n::UnitScale{-1,1,1},
                  n::UnitScale{1,1,std::numeric_limits<double>::infinity()}}) {
    auto out=prior;
    EXPECT_EQ(n::EvaluateSiRawGeometry(Profile(),units,in,&out),n::GeometryStatus::InvalidInput);
    Same(out,prior,true);
  }
  auto huge=in;huge.main_vertices[1].x=std::numeric_limits<double>::max();
  auto out=prior;
  EXPECT_EQ(n::EvaluateSiRawGeometry(Profile(),{.125,8,.5},huge,&out),n::GeometryStatus::NonfiniteResult);
  Same(out,prior,true);
}
TEST(Type25GeometryUnits, NativeHistoryCancellationPrecedesOwnerSiConversion) {
  const double offset=.23478201922188535; // Captured native offset scale; nextafter P is synthetic.
  const double penetration=std::nextafter(offset,std::numeric_limits<double>::infinity());
  auto raw=OracleRaw(Profile(),Quad());raw.geometric_penetration=penetration;
  auto row=History(Quad());row.row.penetration_offset=offset;
  const auto expected=OracleHistory(Profile(),1.,{raw},{row});
  n::NativeGeometryHistory staged,scratch_row;
  n::NativeGeometryFinalResult result,scratch_result;
  n::GeometryHistoryBatch batch{&raw,1,&row,1,&staged,&result,&scratch_row,1,&scratch_result,1};
  ASSERT_EQ(n::FinalizeNativeGeometryHistory(Profile(),1.,batch).status,n::GeometryStatus::Ok);
  Number(result.penetration,expected.results[0].penetration,true);
  ASSERT_GT(result.penetration,0);
  bool witness=false;
  for(double length_unit:{.001,.3,.125}) {
    const double owner_penetration=result.penetration*length_unit;
    const double wrongly_shifted_si=penetration*length_unit-offset*length_unit;
    witness |= owner_penetration!=wrongly_shifted_si;
    EXPECT_EQ(owner_penetration,expected.results[0].penetration*length_unit);
  }
  EXPECT_TRUE(witness);
}

} // namespace type25_geometry_test
