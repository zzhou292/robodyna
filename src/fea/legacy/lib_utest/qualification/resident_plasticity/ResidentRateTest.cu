#include "ResidentPlasticityFixture.h"

namespace resident_plasticity_test {
namespace {
// This fixture's h is 1/1024 s, so a deliberate 10 Hz test cutoff exercises
// persistent filter memory. Actual source 10,000 Hz is separately native-qualified.
constexpr tl::material::TabulatedShellPlasticityRate Rate{true,8000,8,10};
__global__ void CollapseTriangle(fe::NodalPreparedView view) {
  for(unsigned a=0;a<3;++a)
    const_cast<double*>(view.kinematics.position_xyz)[3*4+a]=view.kinematics.position_xyz[3*1+a];
}
}
TEST_F(MixedShellCuda, FilterHistoryPublishesAtomicallyAndRejectsDifferentRateScope) {
  {
    Rig mismatch; ASSERT_TRUE(InitializeRate(mismatch,Rate,true));
    fe::NodalTrialToken token; fe::NodalAssemblyView assembly;
    ASSERT_EQ(mismatch.owner.BeginTrial(&token,&assembly).status,fe::NodalStatus::Ok);
    ASSERT_EQ(mismatch.qeph.AssembleAccepted(assembly).status,q::BatchStatus::Success);
    ASSERT_EQ(mismatch.t3.AssembleAccepted(assembly).status,t::BatchStatus::Success);
    mismatch.Discard();
    EXPECT_EQ(mismatch.publication.Initialize(mismatch.owner,mismatch.qeph,mismatch.t3).status,
        fe::ShellPublicationStatus::InvalidInput);
  }
  Rig r; ASSERT_TRUE(InitializeRate(r,Rate)); ASSERT_TRUE(r.Bind());
  const auto qbytes=r.qeph.allocations().device_bytes,tbytes=r.t3.allocations().device_bytes;
  Staged old_shell; SectionPair old_section;
  ASSERT_TRUE(Accepted(r,old_shell)); ASSERT_TRUE(Sections(r,old_section));
  for(const auto* section:{&old_section.q,&old_section.t}) {
    EXPECT_EQ(section->diagnostics.mean_tangent_ratio,1);
    for(const auto& point:section->history.point) EXPECT_EQ(point.filtered_rate_per_s,0);
  }
  for(unsigned interval=0;interval<4;++interval) {
    Snapshot before; ASSERT_TRUE(Read(r.owner,before));
    Prepared prepared; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,interval),prepared));
    Staged next; ASSERT_TRUE(Evaluate(r,prepared,next));
    SectionPair trial; ASSERT_TRUE(Sections(r,trial,&next));
    CheckHostAdapters(r,prepared,old_shell,old_section,next,trial,Rate);
    EXPECT_GT(trial.q.history.point[0].filtered_rate_per_s,0);
    EXPECT_GT(trial.t.history.point[0].filtered_rate_per_s,0);
    if(interval==1) {
      const auto clean=next; const auto clean_section=trial;
      r.Discard();
      Prepared failure; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,interval),failure));
      q::BatchDiagnostics qd;
      ASSERT_EQ(r.qeph.EvaluateCandidate(failure.view,&qd).status,q::BatchStatus::Success);
      CollapseTriangle<<<1,1,0,failure.view.stream>>>(failure.view);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      t::BatchDiagnostics td;
      EXPECT_EQ(r.t3.EvaluateCandidate(failure.view,&td).status,t::BatchStatus::ElementFailure);
      r.Discard();
      Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(after,before);
      Staged accepted; SectionPair accepted_section;
      ASSERT_TRUE(Accepted(r,accepted)); ExactResults(accepted,old_shell);
      ASSERT_TRUE(Sections(r,accepted_section)); SameSections(accepted_section,old_section);
      ASSERT_TRUE(Prepare(r,PlasticSchedule(r,interval),prepared));
      ASSERT_TRUE(Evaluate(r,prepared,next)); ExactResults(next,clean);
      ASSERT_TRUE(Sections(r,trial,&next)); SameSections(trial,clean_section);
    }
    ASSERT_FALSE(::testing::Test::HasFailure());
    ASSERT_TRUE(Publish(r,prepared,next));
    SectionPair published; ASSERT_TRUE(Sections(r,published)); SameSections(published,trial);
    old_shell=next; old_section=published;
    EXPECT_EQ(r.qeph.allocations().device_bytes,qbytes); EXPECT_EQ(r.t3.allocations().device_bytes,tbytes);
  }
}
} // namespace resident_plasticity_test
