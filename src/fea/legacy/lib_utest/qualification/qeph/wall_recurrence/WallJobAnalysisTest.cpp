#include "WallJobAnalysisTestFixture.h"
#include <cmath>
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace jt=job_test;
TEST(QephWallJobAnalysis, MissingRawDataRetainsEveryProgressSlotWithoutTrustingCompletion) {
  auto raw=jt::Job(2,-8); raw.collection_complete=true;
  std::vector<WallJobProgress> events;
  const auto result=AnalyzeWallRawJob(raw,[&](const WallJobAnalysis& current,WallJobProgress event) {
    EXPECT_TRUE(current.input_valid); EXPECT_EQ(current.dimension,194);
    for(unsigned step=0;step<6;++step) {
      EXPECT_EQ(current.steps[step].h,recurrence::Steps[step]);
      for(unsigned a=0;a<3;++a) EXPECT_EQ(current.steps[step].amplitudes[a].amplitude,recurrence::Amplitudes[a]);
    }
    events.push_back(event);
  });
  ASSERT_EQ(events.size(),37);
  for(unsigned step=0;step<6;++step) {
    for(unsigned a=0;a<3;++a) {
      EXPECT_EQ(events[6*step+a].kind,WallJobProgressKind::Amplitude);
      EXPECT_EQ(events[6*step+a].step,step); EXPECT_EQ(events[6*step+a].index,a);
      EXPECT_FALSE(result.steps[step].amplitudes[a].attempted);
    }
    for(unsigned b=0;b<2;++b) {
      EXPECT_EQ(events[6*step+3+b].kind,WallJobProgressKind::Contact);
      EXPECT_EQ(events[6*step+3+b].step,step); EXPECT_EQ(events[6*step+3+b].index,b);
    }
    EXPECT_EQ(events[6*step+5].kind,WallJobProgressKind::Step);
    EXPECT_EQ(events[6*step+5].step,step);
    EXPECT_FALSE(result.steps[step].complete); EXPECT_FALSE(result.steps[step].passed);
  }
  EXPECT_EQ(events.back().kind,WallJobProgressKind::Finished);
  EXPECT_EQ(result.completed_amplitudes,0); EXPECT_EQ(result.completed_contacts,0); EXPECT_EQ(result.completed_steps,0);
  EXPECT_FALSE(result.complete); EXPECT_FALSE(result.passed); EXPECT_TRUE(raw.collection_complete);
}
TEST(QephWallJobAnalysis, ChangedGridAndIncompleteNativeOperandsCannotBecomeNumericalEvidence) {
  const auto clean=jt::Job();
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault);
    auto raw=clean;
    if(fault==0) raw.cells=2;
    if(fault==1) raw.normal_velocity=-0.0;
    if(fault==2) raw.steps.back().h=std::nextafter(raw.steps.back().h,1.0);
    if(fault==3) raw.normal_velocity=1;
    unsigned events=0;
    const auto result=AnalyzeWallRawJob(raw,[&](const WallJobAnalysis&,WallJobProgress event) {
      ++events; EXPECT_EQ(event.kind,WallJobProgressKind::Finished);
    });
    EXPECT_FALSE(result.input_valid); EXPECT_FALSE(result.complete); EXPECT_FALSE(result.passed); EXPECT_EQ(events,1);
  }
  for(unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);
    auto raw=clean; jt::Native(raw,0,0); raw.collection_complete=true;
    auto& p=raw.steps[0].native[0];
    if(fault==0) p.derivative.amplitude=recurrence::Amplitudes[1];
    if(fault==1) --p.derivative.completed_columns;
    if(fault==2) p.velocity.z=1;
    if(fault==3) p.derivative.full(0,0)=std::numeric_limits<double>::quiet_NaN();
    if(fault==4) p.baseline.conservativeResize(p.baseline.size()-1);
    if(fault==5) raw.steps[0].native_attempted[0]=false;
    if(fault==6) p.derivative.complete=false;
    const auto result=AnalyzeWallRawJob(raw);
    EXPECT_TRUE(result.input_valid); EXPECT_FALSE(result.steps[0].amplitudes[0].input_complete);
    EXPECT_FALSE(result.steps[0].amplitudes[0].branches.complete); EXPECT_EQ(result.completed_amplitudes,0);
    EXPECT_FALSE(result.passed); EXPECT_TRUE(raw.collection_complete);
  }
}
TEST(QephWallJobAnalysis, RetainsFailedFullAnalysesAndRecomputesLocalConsistencyAndContact) {
  auto raw=jt::Job();
  for(unsigned a=0;a<3;++a) jt::Native(raw,0,a);
  // Independent synthetic identity operators are mutually identical but fail
  // the real observer/cache identities. A bad moving baseline must not hide
  // the available failed branch/spectrum/Gram evidence for that amplitude.
  raw.steps[0].native[1].baseline[0]=.001;
  for(unsigned b=0;b<2;++b) {
    raw.steps[0].contact_attempted[b]=true;
    raw.steps[0].contact[b]=jt::Contact(raw,0,b?ContactBranch::Active:ContactBranch::Inactive,
                                     raw.steps[0].native[2].derivative.full);
  }
  unsigned completed_step_callbacks=0;
  const auto result=AnalyzeWallRawJob(raw,[&](const WallJobAnalysis& current,WallJobProgress event) {
    if(event.kind==WallJobProgressKind::Step&&event.step==0) {
      ++completed_step_callbacks;
      EXPECT_TRUE(current.steps[0].complete); EXPECT_FALSE(current.steps[0].passed);
      EXPECT_EQ(current.completed_amplitudes,3); EXPECT_EQ(current.completed_contacts,2);
    }
  });
  EXPECT_EQ(completed_step_callbacks,1); EXPECT_EQ(result.completed_steps,1);
  EXPECT_FALSE(result.complete); EXPECT_FALSE(result.passed);
  for(unsigned a=0;a<3;++a) {
    const auto& analysis=result.steps[0].amplitudes[a];
    ASSERT_TRUE(analysis.complete)<<analysis.diagnostic; EXPECT_FALSE(analysis.passed);
    EXPECT_EQ(analysis.baseline.passed,a!=1);
    EXPECT_EQ(analysis.branches.completed_branches,2); EXPECT_EQ(analysis.branches.completed_events,9);
    EXPECT_GT(analysis.branches.branches[0].identities.observer_error,recurrence::MatrixTolerance);
    EXPECT_EQ(analysis.branches.branches[0].spectrum.eigenvalues.size(),109);
    EXPECT_EQ(analysis.branches.events[8].sequence.weighted.controlling_direction.size(),109);
    EXPECT_TRUE(raw.steps[0].native[a].derivative.full.isIdentity());
  }
  for(unsigned pair=0;pair<2;++pair) {
    EXPECT_TRUE(result.steps[0].native_amplitude_comparisons[pair].passed);
    EXPECT_TRUE(result.steps[0].derived_amplitude_comparisons[pair].passed);
  }
  for(unsigned b=0;b<2;++b) {
    EXPECT_TRUE(result.steps[0].contact[b].passed); EXPECT_FALSE(raw.steps[0].contact[b].passed);
  }
  EXPECT_EQ(raw.steps[0].native[1].baseline[0],.001); EXPECT_FALSE(raw.collection_complete);
}
TEST(QephWallJobAnalysis, CallbackInterruptionStopsAfterDurableFirstAmplitudeEvidence) {
  auto raw=jt::Job(); jt::Native(raw,0,0);
  struct Stop {};
  WallAmplitudeAnalysis saved; unsigned events=0;
  EXPECT_THROW(AnalyzeWallRawJob(raw,[&](const WallJobAnalysis& current,WallJobProgress event) {
    ++events; EXPECT_EQ(event.kind,WallJobProgressKind::Amplitude); EXPECT_EQ(event.step,0); EXPECT_EQ(event.index,0);
    saved=current.steps[0].amplitudes[0];
    throw Stop{};
  }),Stop);
  EXPECT_EQ(events,1); EXPECT_TRUE(saved.complete); EXPECT_FALSE(saved.passed);
  EXPECT_EQ(saved.branches.completed_events,9);
  EXPECT_EQ(saved.branches.branches[1].weighted_spectrum.eigenvalues.size(),109);
  EXPECT_TRUE(raw.steps[0].native[0].derivative.full.isIdentity());
  EXPECT_FALSE(raw.steps[0].native_attempted[1]);
}
} // namespace tl::qualification::qeph::wall_recurrence
