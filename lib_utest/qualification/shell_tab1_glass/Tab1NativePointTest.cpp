#include "Tab1NativeSupport.h"
namespace tab1_test {
TEST(Tab1Native, ExactStarterDefaultsAndWeightedTableBoundariesExtrapolationAndCache) {
  double p[22]{},threshold=0;
  tab1_native_defaults(p,&threshold);
  EXPECT_EQ(threshold,1.e-6);
  EXPECT_EQ(p[1],1.); EXPECT_EQ(p[3],1.); EXPECT_EQ(p[4],0.); EXPECT_EQ(p[5],1.);
  EXPECT_EQ(p[15],0.); EXPECT_EQ(p[16],0.); EXPECT_EQ(p[17],0.); EXPECT_EQ(p[18],0.);
  EXPECT_EQ(p[19],1.); EXPECT_EQ(p[20],-1.); EXPECT_EQ(p[21],1.);
  const auto table=Table();
  std::uint32_t current=0;
  int native_cache=0;
  for(double x:{-.3,std::nextafter(-.3,-1.),std::nextafter(-.3,0.),0.,
      std::nextafter(0.,1.),.3,std::nextafter(.3,1.),1./3.,2./3.,-2./3.,
      -1./3.,std::nextafter(0.,-1.),1.e20,-1.e20}) {
    double expected=0,slope=17,actual=0;
    tab1_native_table(table.triaxiality,table.failure_strain,x,native_cache,&expected,&slope,&native_cache);
    ASSERT_TRUE(fail::EvaluateTab1ConstantTable(table,x,current,actual,current));
    EXPECT_EQ(Bytes(actual),Bytes(expected));
    EXPECT_EQ(current,native_cache); EXPECT_EQ(slope,0.);
  }
}
TEST(Tab1Native, PositiveTinyIncrementLoadUnloadOvershootAndInactiveRetention) {
  const auto table=Table();
  // Resolve the native EM20 denominator boundary through the complete donor,
  // including stresses whose equivalent value rounds to either side of it.
  for(double magnitude:{0.,.5e-20,std::nextafter(1.e-20,0.),1.e-20,
      std::nextafter(1.e-20,1.),2.e-20}) {
    SCOPED_TRACE(magnitude);
    for(double sign:{-1.,1.}) {
      fail::Tab1ConstantFailureHistory base;
      const auto old=NativeHistory(base);
      fail::Tab1ConstantFailureInput in;
      in.current_stress[0]=sign*magnitude;
      in.plastic_strain_increment=.001;
      in.native_evaluation_time_s=1.e-7;
      const double stress[5]{in.current_stress[0],0,0,0,0};
      std::array<double,6> expected{};
      tab1_native_point(table.triaxiality,table.failure_strain,old.data(),1,
          in.plastic_strain_increment,in.native_evaluation_time_s,stress,expected.data());
      fail::Tab1ConstantFailureResult result;
      ASSERT_EQ(fail::UpdateTab1ConstantFailure(table,base,in,result),Status::Ok);
      EXPECT_EQ(Bytes(result.history.damage),Bytes(expected[0]));
      EXPECT_EQ(result.history.failure_time_s,expected[1]);
      EXPECT_EQ(result.history.point_active,expected[2]==1.);
      EXPECT_EQ(Bytes(result.history.maximum_damage),Bytes(expected[3]));
      EXPECT_EQ(result.history.table_segment,expected[4]);
    }
  }
  fail::Tab1ConstantFailureHistory actual;
  auto native=NativeHistory(actual);
  for(unsigned step=0;step<9;++step) {
    fail::Tab1ConstantFailureInput in;
    in.native_evaluation_time_s=(step+1)*1.e-7;
    in.plastic_strain_increment=step==0?std::nextafter(0.,1.):step<3?.001:step==3?.03:0.;
    in.current_stress[0]=(step%2?1.:-1.)*30e6;
    in.current_stress[1]=step==2?30e6:0.;
    in.current_stress[2]=step==1?15e6:0.;
    in.element_active=step<7;
    double stress[5]{in.current_stress[0],in.current_stress[1],in.current_stress[2],0,0};
    std::array<double,6> expected{};
    tab1_native_point(table.triaxiality,table.failure_strain,native.data(),in.element_active?1:0,
        in.plastic_strain_increment,in.native_evaluation_time_s,stress,expected.data());
    fail::Tab1ConstantFailureResult result;
    ASSERT_EQ(fail::UpdateTab1ConstantFailure(table,actual,in,result),Status::Ok);
    Close(result.history.damage,expected[0],0.);
    EXPECT_EQ(result.history.failure_time_s,expected[1]);
    EXPECT_EQ(result.history.point_active,expected[2]==1.);
    Close(result.history.maximum_damage,expected[3],0.);
    EXPECT_EQ(result.history.table_segment,expected[4]); EXPECT_EQ(expected[5],0.);
    if(step==0) EXPECT_GT(result.history.damage,0.);
    if(step>=3) { EXPECT_GT(result.history.damage,1.); EXPECT_EQ(result.history.maximum_damage,1.); }
    actual=result.history;
    std::copy_n(expected.begin(),5,native.begin());
  }
}
} // namespace tab1_test
