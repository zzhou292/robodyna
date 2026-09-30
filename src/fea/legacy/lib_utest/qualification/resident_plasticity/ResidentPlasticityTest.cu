#include "../active_shell_collection/StorageExpectations.h"
#include "ResidentPlasticityFixture.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/t3/T3BatchStorage.h"

namespace resident_plasticity_test {
namespace {
__global__ void CollapseLastTriangle(fe::NodalPreparedView view) {
  for(unsigned a=0;a<3;++a)
    const_cast<double*>(view.kinematics.position_xyz)[3*4+a]=view.kinematics.position_xyz[3*1+a];
}
}
TEST_F(MixedShellCuda, ResidentPlasticSectionsPublishWithBothShellsAcrossLoadHoldReverse) {
  Rig r; ASSERT_TRUE(Initialize(r)); ASSERT_TRUE(r.Bind());
  const auto qa=r.qeph.allocations(),ta=r.t3.allocations();
  ASSERT_EQ(qa.device_allocations,2u); ASSERT_EQ(ta.device_allocations,2u);
  EXPECT_EQ(qa.device_bytes,active_shell_test::QBytes(1,Nodes,3));
  EXPECT_EQ(ta.device_bytes,active_shell_test::TBytes(1,Nodes,3));
  Staged old_shell; SectionPair old_section;
  ASSERT_TRUE(Accepted(r,old_shell)); ASSERT_TRUE(Sections(r,old_section));
  EXPECT_EQ(old_section.q.cumulative_plastic_work_J,0); EXPECT_EQ(old_section.t.cumulative_plastic_work_J,0);
  for(unsigned interval=0;interval<4;++interval) {
    Prepared prepared; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,interval),prepared));
    Staged next; ASSERT_TRUE(Evaluate(r,prepared,next));
    SectionPair trial; ASSERT_TRUE(Sections(r,trial,&next));
    CheckHostAdapters(r,prepared,old_shell,old_section,next,trial);
    SectionPair still_accepted; ASSERT_TRUE(Sections(r,still_accepted)); SameSections(still_accepted,old_section);
    if(interval==0) {
      EXPECT_GT(trial.q.diagnostics.maximum_plastic_strain,0);
      EXPECT_GT(trial.t.diagnostics.maximum_plastic_strain,0);
      EXPECT_GT(trial.q.cumulative_plastic_work_J,0); EXPECT_GT(trial.t.cumulative_plastic_work_J,0);
    }
    ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Publish(r,prepared,next));
    SectionPair published; ASSERT_TRUE(Sections(r,published)); SameSections(published,trial);
    EXPECT_GE(published.q.diagnostics.maximum_plastic_strain,old_section.q.diagnostics.maximum_plastic_strain);
    EXPECT_GE(published.t.diagnostics.maximum_plastic_strain,old_section.t.diagnostics.maximum_plastic_strain);
    EXPECT_EQ(r.qeph.allocations().device_bytes,qa.device_bytes); EXPECT_EQ(r.t3.allocations().device_bytes,ta.device_bytes);
    EXPECT_EQ(r.qeph.allocations().device_allocations,2u); EXPECT_EQ(r.t3.allocations().device_allocations,2u);
    old_shell=next; old_section=published;
  }
  RecordProperty("prescribed_plastic_intervals",4);
}

TEST_F(MixedShellCuda, LateFamilyFailurePreservesSectionsAndRetriesExactly) {
  Rig r; ASSERT_TRUE(Initialize(r)); ASSERT_TRUE(r.Bind());
  Snapshot before; ASSERT_TRUE(Read(r.owner,before));
  Staged old_shell; SectionPair old_section;
  ASSERT_TRUE(Accepted(r,old_shell)); ASSERT_TRUE(Sections(r,old_section));
  Prepared clean_prepared; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),clean_prepared));
  Staged clean; ASSERT_TRUE(Evaluate(r,clean_prepared,clean));
  SectionPair clean_section; ASSERT_TRUE(Sections(r,clean_section,&clean));
  r.Discard();
  Prepared failed; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),failed));
  q::BatchDiagnostics qd; ASSERT_EQ(r.qeph.EvaluateCandidate(failed.view,&qd).status,q::BatchStatus::Success);
  fe::ShellBatchSectionState scratch;
  ASSERT_EQ(r.qeph.CopyPreparedSectionHistory(qd,&scratch,1).status,q::BatchStatus::Success);
  EXPECT_GT(scratch.diagnostics.maximum_plastic_strain,0);
  CollapseLastTriangle<<<1,1,0,failed.view.stream>>>(failed.view);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  t::BatchDiagnostics td; const auto rejected=r.t3.EvaluateCandidate(failed.view,&td);
  ASSERT_EQ(rejected.status,t::BatchStatus::ElementFailure); EXPECT_EQ(rejected.element,0u);
  r.Discard();
  SectionPair after_section; Staged after_shell; Snapshot after;
  ASSERT_TRUE(Sections(r,after_section)); SameSections(after_section,old_section);
  ASSERT_TRUE(Accepted(r,after_shell)); ExactResults(after_shell,old_shell);
  ASSERT_TRUE(Read(r.owner,after)); SameState(after,before);
  const auto saved=Bytes(scratch);
  EXPECT_EQ(r.qeph.CopyPreparedSectionHistory(qd,&scratch,1).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(scratch),saved);
  auto wrong=r.owner.accepted(); ++wrong.epoch;
  EXPECT_EQ(r.qeph.CopyAcceptedSectionHistory(wrong,&scratch,1,&qd).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(scratch),saved);
  Prepared retry_prepared; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,0),retry_prepared));
  Staged retry; ASSERT_TRUE(Evaluate(r,retry_prepared,retry)); ExactResults(retry,clean);
  SectionPair retry_section; ASSERT_TRUE(Sections(r,retry_section,&retry)); SameSections(retry_section,clean_section);
  ASSERT_TRUE(Publish(r,retry_prepared,retry));
  ASSERT_TRUE(Sections(r,after_section)); SameSections(after_section,clean_section);
}

TEST_F(MixedShellCuda, MaterialScopeAndByteCapRejectWithoutChangingTheDefaultAllocation) {
  {
    Rig elastic; ASSERT_TRUE(elastic.Initialize()); ASSERT_TRUE(elastic.Bind());
    EXPECT_EQ(elastic.qeph.allocations().device_bytes,active_shell_test::QBytes(1,Nodes));
    EXPECT_EQ(elastic.t3.allocations().device_bytes,active_shell_test::TBytes(1,Nodes));
    EXPECT_EQ(elastic.qeph.allocations().device_allocations,1u);
    EXPECT_EQ(elastic.t3.allocations().device_allocations,1u);
    fe::ShellBatchPlasticityConfig material{37,47,{CurveX,CurveY,3}};
    q::QephBatchConfig config; config.owner=elastic.owner.accepted(); config.element_count=1;
    config.configuration_id=Configuration; config.qualification_id=Qualification;
    config.usage=q::BatchUsage::PrescribedFields; config.max_device_bytes=active_shell_test::QBytes(1,Nodes);
    q::QephBatch capped;
    EXPECT_EQ(capped.InitializeJoined(config,elastic.binding,material).status,q::BatchStatus::ResourceLimit);
    EXPECT_EQ(capped.allocations().device_allocations,0u);
  }
  for(bool triangle_elastic:{false,true}) {
    Rig r; ASSERT_TRUE(Initialize(r,!triangle_elastic,triangle_elastic));
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    ASSERT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
    ASSERT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::Success);
    r.Discard();
    EXPECT_EQ(r.publication.Initialize(r.owner,r.qeph,r.t3).status,fe::ShellPublicationStatus::InvalidInput);
    EXPECT_EQ(r.owner.accepted().epoch,0u);
  }
}
} // namespace resident_plasticity_test
