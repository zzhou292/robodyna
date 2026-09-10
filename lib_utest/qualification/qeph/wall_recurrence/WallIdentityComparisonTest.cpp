#include "WallRecurrenceAnalysis.h"
#include "WallRecurrenceTestFixture.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
using G=recurrence::Group;
// Analytic external-cache kick followed by kinematic observers. Histories and
// caches are passive identities here: this is an identity oracle, not shell
// constitutive physics or a substitute native matrix.
Eigen::MatrixXd CacheAndDrift(const WallRecurrenceModel& model,double h) {
  const auto& m=model.native(); const auto& d=m.dictionary;
  Eigen::MatrixXd a=Eigen::MatrixXd::Identity(d.size(),d.size());
  for(unsigned e=0;e<m.elements;++e) for(unsigned local=0;local<4;++local) for(unsigned axis=0;axis<3;++axis) {
    const unsigned node=m.connectivity[e][local];
    const auto v=model.coordinate(G::Velocity,node,axis),w=model.coordinate(G::Spin,node,axis);
    const auto f=model.coordinate(G::ForceCache,e,3*local+axis),c=model.coordinate(G::ForceCache,e,12+3*local+axis);
    a(v,f)=static_cast<double>(-static_cast<long double>(h)/m.mass[node]*d[f].scale/d[v].scale);
    a(w,c)=static_cast<double>(-static_cast<long double>(h)/m.inertia[node]*d[c].scale/d[w].scale);
  }
  for(unsigned node=0;node<m.nodes;++node) for(unsigned axis=0;axis<3;++axis) {
    const auto p=model.coordinate(G::Position,node,axis),v=model.coordinate(G::Velocity,node,axis);
    const auto q=model.coordinate(G::OrientationTangent,node,axis),w=model.coordinate(G::Spin,node,axis);
    a.row(p)+=h*d[v].scale/d[p].scale*a.row(v);
    a.row(q)+=h*d[w].scale/d[q].scale*a.row(w);
  }
  return a;
}
WallBranchAnalysis ComparisonFixture() {
  WallBranchAnalysis a; a.h=recurrence::H0; a.complete=true;
  a.metric.diagonal=Eigen::VectorXd::Ones(2);
  a.schedule.total_steps=4096; a.schedule.ordinary_steps=4095;
  for(auto& b:a.branches) {
    b.full=Eigen::MatrixXd::Identity(2,2); b.continuous.complete=true;
    b.continuous.raw.mean_gain=1000; b.continuous.weighted.mean_gain=2;
  }
  for(auto& e:a.events) { e.sequence.complete=true; e.sequence.raw.mean_gain=1000; e.sequence.weighted.mean_gain=2; }
  return a;
}
}
TEST(QephWallIdentities, FullObserverNativeCacheSignAndWallNeutralModesAreIndependentlyChecked) {
  WallRecurrenceModel model; std::string error;
  ASSERT_TRUE(BuildWallRecurrenceModel(2,model,error))<<error;
  const double h=recurrence::H0;
  const auto a=CacheAndDrift(model,h);
  const auto identities=CheckWallStateIdentities(model,h,a);
  ASSERT_TRUE(identities.complete); EXPECT_TRUE(identities.passed)<<identities.diagnostic;
  EXPECT_EQ(identities.neutral_errors.size(),3+model.native().nodes);
  Eigen::MatrixXd active;
  ASSERT_TRUE(BuildContactBranch(model,h,ContactBranch::Active,a,active,error));
  EXPECT_TRUE(CheckWallStateIdentities(model,h,active).passed);
  const auto v=model.coordinate(G::Velocity,0,0),p=model.coordinate(G::Position,0,0);
  const auto f=model.coordinate(G::ForceCache,0,0); auto wrong=a;
  wrong(v,f)=-a(v,f);
  wrong(p,f)=h*model.native().dictionary[v].scale/model.native().dictionary[p].scale*wrong(v,f);
  const auto bad_sign=CheckWallStateIdentities(model,h,wrong);
  EXPECT_TRUE(bad_sign.complete); EXPECT_FALSE(bad_sign.passed);
  EXPECT_GT(bad_sign.cached_kick_error,1e-3); EXPECT_LT(bad_sign.observer_error,recurrence::MatrixTolerance);
  wrong=a; wrong(p,v)+=.001;
  EXPECT_GT(CheckWallStateIdentities(model,h,wrong).observer_error,recurrence::MatrixTolerance);
  // A nonzero passive-work feedback is explicitly measured and retained. It
  // cannot trigger the old zero-position/block-reduction admission shortcut.
  wrong=a; const auto vy=model.coordinate(G::Velocity,0,1),py=model.coordinate(G::Position,0,1);
  const auto work=model.coordinate(G::History,0,34);
  wrong(vy,work)=.001;
  wrong(py,work)=h*model.native().dictionary[vy].scale/model.native().dictionary[py].scale*.001;
  const auto passive=CheckWallStateIdentities(model,h,wrong);
  EXPECT_TRUE(passive.passed); EXPECT_EQ(passive.passive_feedback_max,.001);
  EXPECT_EQ(passive.passive_feedback_column,work); EXPECT_EQ(wrong.rows(),194);
}
TEST(QephWallIdentities, ActualMovingBaselineMustPassBeforeCenteredMatrixUse) {
  WallRecurrenceModel model; std::string error;
  ASSERT_TRUE(BuildWallRecurrenceModel(1,model,error))<<error;
  for(double velocity:VelocityBaselines) {
    MovingMatrixProbe p; p.velocity={velocity,0,0}; p.baseline_complete=true;
    p.baseline=Eigen::VectorXd::Zero(model.native().dictionary.size());
    for(unsigned i=0;i<model.native().dictionary.size();++i) {
      const auto& c=model.native().dictionary[i];
      if(c.group==G::Position&&c.component==0) p.baseline[i]=recurrence::H0*velocity/c.scale;
      if(c.group==G::Velocity&&c.component==0) p.baseline[i]=velocity/c.scale;
    }
    const auto good=CheckWallMovingBaseline(model,recurrence::H0,p);
    ASSERT_TRUE(good.complete); EXPECT_TRUE(good.passed); EXPECT_EQ(good.maximum_error,0);
    const auto work=model.coordinate(G::History,0,34); p.baseline[work]=1e-5;
    const auto bad=CheckWallMovingBaseline(model,recurrence::H0,p);
    EXPECT_TRUE(bad.complete); EXPECT_FALSE(bad.passed); EXPECT_EQ(bad.controlling_coordinate,work);
    EXPECT_EQ(bad.residual[work],1e-5); EXPECT_EQ(p.baseline[work],1e-5);
    p.velocity.y=1; EXPECT_FALSE(CheckWallMovingBaseline(model,recurrence::H0,p).complete);
  }
}
TEST(QephWallComparison, WeightedConsistencyGatesWhileRawDifferencesRemainDiagnostics) {
  auto a=ComparisonFixture(),b=a;
  auto comparison=CompareWallAnalyses(a,b); ASSERT_TRUE(comparison.complete); EXPECT_TRUE(comparison.passed);
  b.events[8].sequence.raw.mean_gain=2000;
  comparison=CompareWallAnalyses(a,b);
  ASSERT_TRUE(comparison.complete); EXPECT_TRUE(comparison.passed); EXPECT_FALSE(comparison.raw_gains[10].passed);
  EXPECT_GT(comparison.raw_gains[10].difference,comparison.raw_gains[10].budget);
  b.events[8].sequence.weighted.mean_gain=2.1;
  comparison=CompareWallAnalyses(a,b);
  ASSERT_TRUE(comparison.complete); EXPECT_FALSE(comparison.passed); EXPECT_FALSE(comparison.weighted_gains[10].passed);
  EXPECT_TRUE(comparison.weighted_gains[9].passed); EXPECT_TRUE(comparison.matrices[0].passed);
  b=a; b.branches[1].full(1,0)=1e-6;
  comparison=CompareWallAnalyses(a,b); ASSERT_TRUE(comparison.complete); EXPECT_FALSE(comparison.passed);
  EXPECT_FALSE(comparison.matrices[1].passed);
  b=a; b.events[0].window.entry_shift=1; EXPECT_FALSE(CompareWallAnalyses(a,b).complete);
  b=a; b.metric.diagonal[0]=.5; EXPECT_FALSE(CompareWallAnalyses(a,b).complete);
  b=a; b.h*=2; EXPECT_FALSE(CompareWallAnalyses(a,b).complete);
  EXPECT_FALSE(CompareWallGains(std::numeric_limits<double>::quiet_NaN(),1).complete);
  EXPECT_FALSE(CompareWallGains(-1,1).complete);
}
TEST(QephWallComparison, FrozenSelectorKeepsFactorTwoMarginAndNeverFitsLargerStep) {
  std::array<bool,6> pass{{true,true,true,true,true,true}};
  EXPECT_EQ(SelectWallScreenStep(pass),recurrence::H0);
  pass[5]=false; EXPECT_EQ(SelectWallScreenStep(pass),recurrence::H0);
  pass[4]=false; EXPECT_EQ(SelectWallScreenStep(pass),recurrence::H0/2);
  for(unsigned i=0;i<4;++i) {
    auto missing=pass; missing[i]=false; EXPECT_EQ(SelectWallScreenStep(missing),0);
  }
  WallRecurrenceModel empty;
  const auto failed=AnalyzeWallBranches(empty,recurrence::H0,Eigen::MatrixXd::Identity(2,2));
  EXPECT_FALSE(failed.complete); EXPECT_FALSE(failed.passed); EXPECT_FALSE(failed.diagnostic.empty());
  EXPECT_EQ(failed.completed_branches,0u); EXPECT_EQ(failed.completed_events,0u);
}
} // namespace tl::qualification::qeph::wall_recurrence
