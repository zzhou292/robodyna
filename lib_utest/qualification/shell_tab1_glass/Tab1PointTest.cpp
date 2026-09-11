#include "Tab1Fixture.h"
#include <cmath>
#include <limits>
namespace tab1_test {
TEST(Tab1Point, NativeTableWeightedArithmeticAndBidirectionalSegmentIdentity) {
  const auto table=Table();
  std::uint32_t segment=0;
  for(double x:{0.,.3,1./3.,2./3.,-.3,-1./3.,-2./3.,
      std::nextafter(0.,1.),std::nextafter(0.,-1.),0.,1.e20,-1.e20}) {
    const unsigned i=x<=0?0:1;
    const double r=(table.triaxiality[i+1]-x)/(table.triaxiality[i+1]-table.triaxiality[i]);
    const double expected=r*.015+(1.-r)*.015;
    double actual=-17.;
    ASSERT_TRUE(fail::EvaluateTab1ConstantTable(table,x,segment,actual,segment));
    EXPECT_DOUBLE_EQ(actual,expected);
    EXPECT_EQ(segment,i+1);
  }
  double value=13.;
  const auto old_segment=segment;
  EXPECT_FALSE(fail::EvaluateTab1ConstantTable(table,std::numeric_limits<double>::max(),segment,value,segment));
  EXPECT_EQ(value,13.); EXPECT_EQ(segment,old_segment);
}
TEST(Tab1Point, RetainedDamageOvershootsWhileDisplayCapsAndInactiveStateStaysNative) {
  fail::Tab1ConstantFailureHistory h;
  fail::Tab1ConstantFailureInput in;
  in.plastic_strain_increment=std::nextafter(0.,1.);
  in.native_evaluation_time_s=1.e-7;
  fail::Tab1ConstantFailureResult r;
  ASSERT_EQ(fail::UpdateTab1ConstantFailure(Table(),h,in,r),Status::Ok);
  EXPECT_GT(r.history.damage,0.);
  EXPECT_TRUE(r.history.point_active);
  h=r.history;
  in.plastic_strain_increment=.03;
  in.native_evaluation_time_s=2.e-7;
  ASSERT_EQ(fail::UpdateTab1ConstantFailure(Table(),h,in,r),Status::Ok);
  EXPECT_GE(r.history.damage,2.);
  EXPECT_EQ(r.history.maximum_damage,1.);
  EXPECT_FALSE(r.history.point_active);
  EXPECT_TRUE(r.failed_now);
  EXPECT_EQ(r.history.failure_time_s,2.e-7);
  h=r.history;
  in.native_evaluation_time_s=3.e-7;
  in.current_stress[0]=-30e6;
  ASSERT_EQ(fail::UpdateTab1ConstantFailure(Table(),h,in,r),Status::Ok);
  SameFailureHistory(h,r.history);
  EXPECT_FALSE(r.failed_now);
  h={}; h.damage=h.maximum_damage=.25; h.table_segment=2;
  in.element_active=false;
  ASSERT_EQ(fail::UpdateTab1ConstantFailure(Table(),h,in,r),Status::Ok);
  SameFailureHistory(h,r.history);
}
TEST(Tab1Point, InvalidDeclarationsHistoriesAndLateOverflowPreserveOutputThenRetry) {
  for(unsigned fault=0;fault<7;++fault) {
    auto table=Table();
    fail::Tab1ConstantFailureHistory h;
    fail::Tab1ConstantFailureInput in;
    in.plastic_strain_increment=.001;
    in.native_evaluation_time_s=1.e-7;
    fail::Tab1ConstantFailureResult r;
    r.history.maximum_damage=.5;
    const auto bytes=Bytes(r);
    Status expected=Status::InvalidParameters;
    if(fault==0)table.failure_strain=0;
    if(fault==1)table.triaxiality[2]=0;
    if(fault==2){h.table_segment=3;expected=Status::InvalidHistory;}
    if(fault==3){h.damage=1.1;expected=Status::InvalidHistory;}
    if(fault==4){in.plastic_strain_increment=-1;expected=Status::InvalidIncrement;}
    if(fault==5){in.current_stress[2]=1.e200;expected=Status::NonfiniteResult;}
    if(fault==6){in.plastic_strain_increment=std::numeric_limits<double>::max();expected=Status::NonfiniteResult;}
    EXPECT_EQ(fail::UpdateTab1ConstantFailure(table,h,in,r),expected);
    EXPECT_EQ(Bytes(r),bytes);
    h={}; in={};
    ASSERT_EQ(fail::UpdateTab1ConstantFailure(Table(),h,in,r),Status::Ok);
  }
}
} // namespace tab1_test
