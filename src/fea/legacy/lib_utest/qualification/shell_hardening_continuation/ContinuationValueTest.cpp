#include "ContinuationFixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace continuation_test {
TEST(HardeningContinuation, OriginalEndpointsUseLastSegmentWithoutChangingTables) {
  const double at_one[]{455000000.,493999999.9999999,460000000.};
  for(std::size_t index=0;index<Curves.size();++index) {
    const auto c=Curves[index];
    const auto p=Prepare(c);
    ASSERT_TRUE(sec::ValidLayeredJ2Parameters(p));
    EXPECT_EQ(p.curve.count,c.count);
    EXPECT_EQ(p.curve.plastic_strain,c.x);
    EXPECT_EQ(p.curve.yield_stress_pa,c.y);
    const auto last=c.count-1;
    const auto slope=(c.y[last]-c.y[last-1])/(c.x[last]-c.x[last-1]);
    for(double x:{std::nextafter(c.x[last],0.),c.x[last],std::nextafter(c.x[last],1.),1.}) {
      History base; base.plastic_strain=x;
      Result out;
      ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,base,Increment(p,0),out),Status::Ok);
      EXPECT_DOUBLE_EQ(out.yield_before_pa,c.y[last-1]+slope*(x-c.x[last-1]));
      EXPECT_EQ(out.history.plastic_strain,x);
      if(x==1.) EXPECT_DOUBLE_EQ(out.yield_before_pa,at_one[index]);
    }
  }
}
TEST(HardeningContinuation, StrictDefaultAndInvalidPolicyOrCapLeaveOutputUnchanged) {
  for(const auto c:Curves) {
    Parameters strict;
    ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{c.x,c.y,c.count},strict),Status::Ok);
    EXPECT_EQ(strict.continuation,Policy::StrictDomain);
    auto p=Prepare(c);
    History base; base.plastic_strain=c.x[c.count-1];
    Result out; out.history.plastic_strain=123.;
    const auto old=Bytes(out);
    EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(strict,base,Increment(p),out),Status::CurveDomainExceeded);
    EXPECT_EQ(Bytes(out),old);
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,base,Increment(p),out),Status::Ok);
    ASSERT_GT(out.history.plastic_strain,c.x[c.count-1]);
    const auto accepted=out.history;
    const auto saved=Bytes(out);
    EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(strict,accepted,Increment(p,0),out),Status::CurveDomainExceeded);
    EXPECT_EQ(Bytes(out),saved);
    p.continuation=static_cast<Policy>(255);
    EXPECT_FALSE(sec::ValidLayeredJ2Parameters(p));
    EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,base,Increment(p),out),Status::InvalidParameters);
    EXPECT_EQ(Bytes(out),saved);
    p=Prepare(c); base.plastic_strain=static_cast<double>(1e20f);
    EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,base,Increment(p,0),out),Status::CurveDomainExceeded);
    EXPECT_EQ(Bytes(out),saved);
    base.plastic_strain=1.; auto bad=Increment(p);
    bad.strain_increment[4]=std::numeric_limits<double>::max();
    EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,base,bad,out),Status::NonfiniteResult);
    EXPECT_EQ(Bytes(out),saved);
    EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,base,Increment(p,0),out),Status::Ok);
    const auto old_parameters=Bytes(p);
    EXPECT_EQ(mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,p.curve,{},
        static_cast<Policy>(255),p),Status::InvalidParameters);
    EXPECT_EQ(Bytes(p),old_parameters);
  }
}
TEST(HardeningContinuation, InterpolationOverflowAndAnalyticPolicyMisuseAreRejected) {
  const double x[]{0.,1.},y[]{1.,1.e300};
  Parameters p;
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{x,y,2},{},Policy::NativeLastSegment,p),Status::Ok);
  History base; base.plastic_strain=1.e19;
  Result out; out.history.plastic_strain=123.; const auto old=Bytes(out);
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,base,Increment(p,0),out),Status::InvalidCurve);
  EXPECT_EQ(Bytes(out),old);
  ASSERT_EQ(mat::PrepareLinearLaw44ShellPlasticity(200e9,.3,7890,{220e6,1e9},{true,40,5,100},p),Status::Ok);
  p.continuation=Policy::NativeLastSegment;
  EXPECT_FALSE(sec::ValidLayeredJ2Parameters(p));
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(p,{},Increment(p,0),out),Status::InvalidParameters);
  EXPECT_EQ(Bytes(out),old);
}
} // namespace continuation_test
