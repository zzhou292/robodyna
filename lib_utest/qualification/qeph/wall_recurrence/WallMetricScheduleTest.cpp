#include "WallSwitchingSchedule.h"
#include "WallRecurrenceTestFixture.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
namespace r=recurrence;
using test::Near;
}
TEST(QephWallMetricSchedule, FullDictionaryMetricIsFixedPositiveAndInvertible) {
  for(unsigned cells:{1u,2u}) {
    WallRecurrenceModel model; std::string error;
    ASSERT_TRUE(BuildWallRecurrenceModel(cells,model,error))<<error;
    WallStateMetric metric; ASSERT_TRUE(BuildWallStateMetric(model,metric,error))<<error;
    ASSERT_EQ(metric.diagonal.size(),cells==1?109:194);
    const long double c=std::sqrt(static_cast<long double>(r::Young)/r::Density);
    const long double eta=ScreenHorizon*c/r::Length,wt=1/(1+eta);
    Near(metric.eta,eta); Near(metric.tangential_weight,wt);
    Near(metric.normal_weight,std::max(wt,model.maximum_frequency()*r::Length/c));
    for(unsigned i=0;i<model.native().dictionary.size();++i) {
      const auto& d=model.native().dictionary[i];
      const double expected=d.group==r::Group::Position?(d.component==0?metric.normal_weight:metric.tangential_weight):
        (d.group==r::Group::OrientationTangent?metric.tangential_weight:1.);
      EXPECT_EQ(metric.diagonal[i],expected);
    }
    EXPECT_GT(metric.diagonal.minCoeff(),0); EXPECT_LE(metric.condition,MaximumMetricCondition);
    Eigen::MatrixXd a=Eigen::MatrixXd::Identity(metric.diagonal.size(),metric.diagonal.size());
    a(0,metric.diagonal.size()-1)=.125; const auto saved=a;
    ASSERT_TRUE(ApplyWallStateMetric(a,metric.diagonal,a,error))<<error;
    Near(a(0,a.cols()-1),.125L*metric.diagonal[0]/metric.diagonal.tail(1)[0]);
    const Eigen::VectorXd inverse=metric.diagonal.cwiseInverse();
    ASSERT_TRUE(ApplyWallStateMetric(a,inverse,a,error))<<error;
    EXPECT_LE((a-saved).cwiseAbs().maxCoeff(),2e-12);
  }
}
TEST(QephWallMetricSchedule, SingularConditionOverflowAndMalformedInputsPreserveOutput) {
  Eigen::MatrixXd a=Eigen::MatrixXd::Identity(2,2),out=Eigen::MatrixXd::Constant(3,3,17);
  const auto saved=out; std::string error;
  Eigen::VectorXd d=Eigen::VectorXd::Ones(2);
  for(double bad:{0.,-1.,1e-9,std::numeric_limits<double>::quiet_NaN()}) {
    d[0]=bad; EXPECT_FALSE(ApplyWallStateMetric(a,d,out,error)); EXPECT_EQ(out,saved);
  }
  d<<1,2; a(1,0)=std::numeric_limits<double>::max();
  EXPECT_FALSE(ApplyWallStateMetric(a,d,out,error)); EXPECT_EQ(out,saved);
  a=Eigen::MatrixXd::Identity(2,2); d.setOnes();
  ASSERT_TRUE(ApplyWallStateMetric(a,d,out,error)); EXPECT_EQ(out,a);
  WallRecurrenceModel empty; WallStateMetric metric; metric.eta=17;
  EXPECT_FALSE(BuildWallStateMetric(empty,metric,error)); EXPECT_EQ(metric.eta,17);
}
TEST(QephWallMetricSchedule, ScalarEpochOneScheduleMatchesIndependentSecondOrderPositions) {
  WallRecurrenceModel model; std::string error;
  ASSERT_TRUE(BuildWallRecurrenceModel(1,model,error))<<error;
  for(double h:r::Steps) {
    SCOPED_TRACE(h);
    WallSwitchingSchedule s; ASSERT_TRUE(BuildWallSwitchingSchedule(model,h,s,error))<<error;
    ASSERT_TRUE(s.complete); ASSERT_TRUE(s.passed);
    EXPECT_EQ(s.ordinary_steps+1,s.total_steps); ASSERT_EQ(s.scalar.size(),s.total_steps+1);
    EXPECT_EQ(s.scalar[0].position,-InitialGap); EXPECT_EQ(s.scalar[0].velocity,ImpactSpeed);
    EXPECT_EQ(s.scalar[1].position,-InitialGap+h*ImpactSpeed);
    EXPECT_EQ(s.scalar[1].velocity,ImpactSpeed); EXPECT_FALSE(s.scalar[1].active);
    // Independent two-position recurrence, no kick/velocity implementation.
    long double old=-static_cast<long double>(InitialGap),x=old+static_cast<long double>(h)*ImpactSpeed;
    const long double hw=static_cast<long double>(h)*model.maximum_frequency();
    unsigned entry=0,exit=0;
    for(unsigned epoch=1;epoch<s.total_steps;++epoch) {
      if(x>=0&&entry==0) entry=epoch;
      if(x<0&&entry!=0&&exit==0) exit=epoch;
      const long double next=2*x-old-hw*hw*std::max(x,0.L);
      // The prospective scalar witness is a physical position, compared at the
      // inherited normalized 2e-12 scale; its signs also independently bind events.
      Near(s.scalar[epoch].position,x);
      EXPECT_EQ(s.scalar[epoch].active,x>=0);
      old=x; x=next;
    }
    Near(s.scalar.back().position,x); EXPECT_EQ(s.entry_base_epoch,entry); EXPECT_EQ(s.exit_base_epoch,exit);
    unsigned index=0;
    for(int de:{-1,0,1}) for(int dx:{-1,0,1}) {
      const auto& w=s.windows[index++];
      EXPECT_EQ(static_cast<long long>(w.entry_base_epoch),static_cast<long long>(entry)+de);
      EXPECT_EQ(static_cast<long long>(w.exit_base_epoch),static_cast<long long>(exit)+dx);
      EXPECT_EQ(w.inactive_before+w.active+w.inactive_after,s.ordinary_steps);
      // Literal base masks count each transition epoch1..H/h-1 exactly once.
      unsigned before=0,active=0,after=0;
      for(unsigned epoch=1;epoch<s.total_steps;++epoch) {
        if(epoch<w.entry_base_epoch) ++before;
        else if(epoch<w.exit_base_epoch) ++active;
        else ++after;
      }
      EXPECT_EQ(before,w.inactive_before); EXPECT_EQ(active,w.active); EXPECT_EQ(after,w.inactive_after);
    }
  }
}
TEST(QephWallMetricSchedule, WindowBoundaryRejectionStagesAllNineThenRetries) {
  EXPECT_TRUE(WallScalarContactActive(0.)); EXPECT_TRUE(WallScalarContactActive(-0.));
  EXPECT_FALSE(WallScalarContactActive(-std::numeric_limits<double>::denorm_min()));
  EXPECT_TRUE(WallScalarContactActive(std::numeric_limits<double>::denorm_min()));
  std::array<WallSwitchingWindow,9> out{}; std::string error;
  ASSERT_TRUE(BuildWallSwitchingWindows(12,4,8,out,error)); const auto saved=out;
  const std::array<std::array<unsigned,2>,4> bad{{{1,8},{4,12},{4,6},{8,4}}};
  for(const auto& pair:bad) {
    EXPECT_FALSE(BuildWallSwitchingWindows(12,pair[0],pair[1],out,error));
    for(unsigned i=0;i<9;++i) {
      EXPECT_EQ(out[i].entry_base_epoch,saved[i].entry_base_epoch);
      EXPECT_EQ(out[i].exit_base_epoch,saved[i].exit_base_epoch);
    }
  }
  // At an extreme valid window the first or last inactive run may be empty.
  ASSERT_TRUE(BuildWallSwitchingWindows(12,2,11,out,error));
  EXPECT_EQ(out.front().inactive_before,0u); EXPECT_EQ(out.back().inactive_after,0u);
  WallRecurrenceModel empty; WallSwitchingSchedule held; held.h=17;
  EXPECT_FALSE(BuildWallSwitchingSchedule(empty,r::H0,held,error)); EXPECT_EQ(held.h,17);
  const auto failed=AnalyzeWallSwitchingSchedule(empty,r::H0);
  EXPECT_FALSE(failed.complete); EXPECT_FALSE(failed.passed); EXPECT_FALSE(failed.diagnostic.empty());
}
} // namespace tl::qualification::qeph::wall_recurrence
