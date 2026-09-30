#include "AnalyticTestSupport.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"
#include <cmath>
#include <limits>

namespace analytic_test {
TEST(Law44Analytic, OriginalTangentConversionAndVirginSlopeAreExplicit) {
  for(const auto source:Sources) {
    const auto p=Prepare(source);
    const long double e=source.e,a=source.etan;
    const auto expected=static_cast<double>((a*e/(e-a))*1e6L);
    EXPECT_NEAR(p.plastic_hardening_pa,expected,4*std::numeric_limits<double>::epsilon()*expected);
    EXPECT_EQ(p.linear.initial_yield_pa,source.sigy*1e6);
    EXPECT_EQ(p.hardening,Kind::LinearLaw44); EXPECT_EQ(p.curve.plastic_strain,nullptr);
    EXPECT_EQ(p.curve.yield_stress_pa,nullptr); EXPECT_EQ(p.curve.count,0u);
    ASSERT_TRUE(tl::fea::sections::ValidLayeredJ2Parameters(p));
    const auto in=Increment(p); Result first;
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,{},in,first),Status::Ok);
    ASSERT_GT(first.plastic_increment,0); EXPECT_DOUBLE_EQ(first.tangent_ratio,.5);
    History yielded; yielded.plastic_strain=.001; Result next;
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,yielded,in,next),Status::Ok);
    ASSERT_GT(next.plastic_increment,0); EXPECT_LT(next.tangent_ratio,.1);
    EXPECT_GT(std::abs(first.history.stress[0]-next.history.stress[0]),1e6);
  }
}
TEST(Law44Analytic, InvalidCoefficientModeAndLateArithmeticLeaveOutputsUnchanged) {
  auto p=Prepare(); const auto held=Bytes(p);
  for(const auto bad:std::array<mat::Law44LinearHardening,5>{{{0,1e6},{1e6,-1},{1e6,p.young_pa},
      {std::numeric_limits<double>::infinity(),1},{1e6,std::numeric_limits<double>::quiet_NaN()}}}) {
    EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(p.young_pa,.3,1415,bad,Rate,p),Status::InvalidParameters);
    EXPECT_EQ(Bytes(p),held);
  }
  EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(1e300,.3,1415,{1e6,1e299},Rate,p),Status::InvalidParameters);
  EXPECT_EQ(Bytes(p),held); // Converter product overflow, although rearranging it would be finite.
  EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(p.young_pa,.3,1415,p.linear,{},p),Status::InvalidParameters);
  EXPECT_EQ(Bytes(p),held);
  Result result; result.history.stress[4]=17; const auto old=Bytes(result); auto in=Increment(p);
  auto damaged=p; damaged.plastic_hardening_pa=std::nextafter(p.plastic_hardening_pa,INFINITY);
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(damaged,{},in,result),Status::InvalidParameters); EXPECT_EQ(Bytes(result),old);
  damaged=p; damaged.curve.count=2;
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(damaged,{},in,result),Status::InvalidParameters); EXPECT_EQ(Bytes(result),old);
  damaged=p; damaged.hardening=static_cast<Kind>(127);
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(damaged,{},in,result),Status::InvalidParameters); EXPECT_EQ(Bytes(result),old);
  in.strain_increment[4]=std::numeric_limits<double>::max();
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,{},in,result),Status::NonfiniteResult); EXPECT_EQ(Bytes(result),old);
  in=Increment(p); History history; history.plastic_strain=std::numeric_limits<double>::max();
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,history,in,result),Status::HardeningDomainExceeded); EXPECT_EQ(Bytes(result),old);
  EXPECT_EQ(mat::UpdateTabulatedShellPlasticity(p,{},in,result),Status::InvalidParameters); EXPECT_EQ(Bytes(result),old);
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,{},in,result),Status::Ok);
}
TEST(Law44Analytic, LegacyAggregatesAndTabulatedDispatchRetainTheirArithmetic) {
  double x[]{0,.1,.3},y[]{20e6,21e6,23e6}; Parameters p;
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(1e9,.3,1415,{x,y,3},Rate,p),Status::Ok);
  EXPECT_EQ(p.hardening,Kind::Tabulated); EXPECT_EQ(p.linear.initial_yield_pa,0); EXPECT_EQ(p.plastic_hardening_pa,0);
  History a,b; auto in=Increment(p);
  for(unsigned i=0;i<32;++i) {
    in.strain_increment[0]=i<16?.002:-.002;
    Result legacy,common;
    ASSERT_EQ(mat::UpdateTabulatedShellPlasticity(p,a,in,legacy),Status::Ok);
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,b,in,common),Status::Ok);
    EXPECT_EQ(Bytes(legacy),Bytes(common)); a=legacy.history; b=common.history;
  }
  // A pre-extension positional parameter aggregate must still bind the same fields.
  Parameters old{{x,y,3},1e9,.3,1415,p.shear_modulus,p.a11,p.a12,p.three_g,p.sound_speed,
      Rate,p.inverse_rate_c,p.inverse_rate_p,p.angular_cutoff_per_s};
  EXPECT_EQ(old.hardening,Kind::Tabulated); EXPECT_TRUE(tl::fea::sections::ValidLayeredJ2Parameters(old));
}
} // namespace analytic_test
