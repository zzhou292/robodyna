#include "ContinuationNativeSupport.h"
#include <vector>

namespace continuation_test {
extern "C" void continuation_curve(int,const double*,int,const double*,double*,double*,int*);
TEST(HardeningContinuationNative, OriginalKnotsAndBeyondUseCompleteCachedVinter) {
  for(const auto c:Curves) {
    SCOPED_TRACE(c.id);
    const auto p=Prepare(c);
    std::vector<double> points(2*(c.count+1),0),probes;
    for(unsigned i=0;i<c.count;++i) {
      points[2*(i+1)]=c.x[i]; points[2*(i+1)+1]=c.y[i];
      if(i) probes.push_back(std::nextafter(c.x[i],0.));
      probes.push_back(c.x[i]);
      probes.push_back(std::nextafter(c.x[i],1.));
    }
    probes.push_back(1.); probes.push_back(1.25);
    std::vector<double> values(probes.size()),slopes(probes.size());
    std::vector<int> segments(probes.size());
    continuation_curve(c.count,points.data(),probes.size(),probes.data(),values.data(),slopes.data(),segments.data());
    for(std::size_t i=0;i<probes.size();++i) {
      double value=0,slope=0;
      ASSERT_TRUE(mat::tabulated_shell_detail::CurveValue(p.curve,probes[i],value,slope));
      EXPECT_EQ(value,values[i]); EXPECT_EQ(slope,slopes[i]);
      if(i) EXPECT_GE(segments[i],segments[i-1]);
    }
    EXPECT_EQ(segments.back(),static_cast<int>(c.count-1));
  }
}
TEST(HardeningContinuationNative, BoundaryCrossingLoadingAndElasticUnloadingRetainIndependentHistories) {
  for(const auto c:Curves) for(bool rate:{false,true}) {
    SCOPED_TRACE(c.id);
    SCOPED_TRACE(rate);
    const auto p=Prepare(c,rate);
    for(double start:{std::nextafter(c.x[c.count-1],0.),c.x[c.count-1],
                      std::nextafter(c.x[c.count-1],1.)}) {
      History history; history.plastic_strain=start;
      auto input=Increment(p,0.);
      auto oracle=NativeInput(p,history,input);
      unsigned yielded=0,unloaded=0;
      for(unsigned step=0;step<14;++step) {
        const double dx=step==0?0.:step<10?.004:-1.e-5;
        input=Increment(p,dx);
        std::copy_n(input.strain_increment,5,oracle.strain_increment.begin());
        native::Result expected;
        ASSERT_TRUE(native::Evaluate(oracle,expected));
        Result actual;
        ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,history,input,actual),Status::Ok);
        ComparePoint(actual,expected);
        yielded+=actual.plastic_increment>0;
        if(step>=10) { EXPECT_EQ(actual.plastic_increment,0.); ++unloaded; }
        history=actual.history; Accept(oracle,expected);
      }
      EXPECT_GT(history.plastic_strain,c.x[c.count-1]);
      EXPECT_GE(yielded,8u); EXPECT_EQ(unloaded,4u);
      oracle.continuation=native::CurveContinuation::StrictDomain;
      native::Result unchanged; unchanged.plastic_strain=123.; const auto before=Bytes(unchanged);
      EXPECT_FALSE(native::Evaluate(oracle,unchanged)); EXPECT_EQ(Bytes(unchanged),before);
    }
  }
}
} // namespace continuation_test
