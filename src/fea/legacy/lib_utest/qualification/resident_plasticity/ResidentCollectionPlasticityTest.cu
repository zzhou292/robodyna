#include "../active_shell_collection/StorageExpectations.h"
#include "ResidentCollectionFixture.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/t3/T3BatchStorage.h"

namespace resident_plasticity_test {
namespace {
__global__ void CollapseCollectionTriangle(fe::NodalPreparedView view) {
  for(unsigned a=0;a<3;++a)
    const_cast<double*>(view.kinematics.position_xyz)[3*4+a]=view.kinematics.position_xyz[3*1+a];
}
}
TEST_F(MixedShellCuda, CollectionMaterialsOwnCurvesYieldAndPublishWithoutAllocationGrowth) {
  Rig r; fe::ShellBatchPlasticityBinding oracle;
  ASSERT_TRUE(InitializeCollection(r,oracle)); ASSERT_TRUE(r.Bind());
  const auto qa=r.qeph.allocations(),ta=r.t3.allocations();
  ASSERT_EQ(qa.device_allocations,2u); ASSERT_EQ(ta.device_allocations,2u);
  EXPECT_EQ(qa.device_bytes,active_shell_test::QBytes(1,Nodes,6));
  EXPECT_EQ(ta.device_bytes,active_shell_test::TBytes(1,Nodes,6));
  Staged old_shell; SectionPair old_section;
  ASSERT_TRUE(Accepted(r,old_shell)); ASSERT_TRUE(Sections(r,old_section));
  for(unsigned interval=0;interval<4;++interval) {
    Prepared p; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,interval),p));
    Staged next; ASSERT_TRUE(Evaluate(r,p,next,interval%2));
    SectionPair trial; ASSERT_TRUE(Sections(r,trial,&next));
    CheckHostAdapters(r,p,old_shell,old_section,next,trial,oracle);
    SectionPair held; ASSERT_TRUE(Sections(r,held)); SameSections(held,old_section);
    if(interval==0) {
      EXPECT_GT(trial.q.diagnostics.maximum_plastic_strain,0);
      EXPECT_GT(trial.t.diagnostics.maximum_plastic_strain,0);
      EXPECT_GT(trial.q.cumulative_plastic_work_J,0); EXPECT_GT(trial.t.cumulative_plastic_work_J,0);
      EXPECT_EQ(trial.q.history.point[0].filtered_rate_per_s,0);
      EXPECT_GT(trial.t.history.point[0].filtered_rate_per_s,0);
    }
    ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Publish(r,p,next));
    SectionPair published; ASSERT_TRUE(Sections(r,published)); SameSections(published,trial);
    EXPECT_EQ(r.qeph.allocations().device_bytes,qa.device_bytes); EXPECT_EQ(r.t3.allocations().device_bytes,ta.device_bytes);
    EXPECT_EQ(r.qeph.allocations().device_allocations,2u); EXPECT_EQ(r.t3.allocations().device_allocations,2u);
    old_shell=next; old_section=published;
  }
}
TEST_F(MixedShellCuda, CollectionLateFailurePreservesYieldedHistoriesAndRetriesExactly) {
  Rig r; fe::ShellBatchPlasticityBinding oracle;
  ASSERT_TRUE(InitializeCollection(r,oracle)); ASSERT_TRUE(r.Bind());
  for(unsigned interval=0;interval<2;++interval) {
    Prepared first; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,interval),first));
    Staged yielded; ASSERT_TRUE(Evaluate(r,first,yielded)); ASSERT_TRUE(Publish(r,first,yielded));
  }
  Snapshot before; ASSERT_TRUE(Read(r.owner,before));
  Staged old_shell; SectionPair old_section;
  ASSERT_TRUE(Accepted(r,old_shell)); ASSERT_TRUE(Sections(r,old_section));
  ASSERT_GT(old_section.q.diagnostics.maximum_plastic_strain,0);
  ASSERT_GT(old_section.t.diagnostics.maximum_plastic_strain,0);
  ASSERT_GT(old_section.q.cumulative_plastic_work_J,0); ASSERT_GT(old_section.t.cumulative_plastic_work_J,0);
  Prepared clean_p; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,2),clean_p));
  Staged clean; ASSERT_TRUE(Evaluate(r,clean_p,clean));
  SectionPair clean_section; ASSERT_TRUE(Sections(r,clean_section,&clean)); r.Discard();
  Prepared failed; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,2),failed));
  q::BatchDiagnostics qd; ASSERT_EQ(r.qeph.EvaluateCandidate(failed.view,&qd).status,q::BatchStatus::Success);
  fe::ShellBatchSectionState trial;
  ASSERT_EQ(r.qeph.CopyPreparedSectionHistory(qd,&trial,1).status,q::BatchStatus::Success);
  ASSERT_GT(trial.diagnostics.maximum_plastic_strain,0);
  CollapseCollectionTriangle<<<1,1,0,failed.view.stream>>>(failed.view);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  t::BatchDiagnostics td; EXPECT_EQ(r.t3.EvaluateCandidate(failed.view,&td).status,t::BatchStatus::ElementFailure);
  r.Discard();
  SectionPair held; Staged held_shell; Snapshot after;
  ASSERT_TRUE(Sections(r,held)); SameSections(held,old_section);
  ASSERT_TRUE(Accepted(r,held_shell)); ExactResults(held_shell,old_shell);
  ASSERT_TRUE(Read(r.owner,after)); SameState(after,before);
  const auto saved=Bytes(trial);
  EXPECT_EQ(r.qeph.CopyPreparedSectionHistory(qd,&trial,1).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(Bytes(trial),saved);
  Prepared retry_p; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,2),retry_p));
  Staged retry; ASSERT_TRUE(Evaluate(r,retry_p,retry)); ExactResults(retry,clean);
  SectionPair retry_section; ASSERT_TRUE(Sections(r,retry_section,&retry)); SameSections(retry_section,clean_section);
  ASSERT_TRUE(Publish(r,retry_p,retry)); ASSERT_TRUE(Sections(r,held)); SameSections(held,clean_section);
}
TEST_F(MixedShellCuda, CollectionPublicationRejectsOtherFamilyChangeAndLegacyMix) {
  for(bool legacy_triangle:{false,true}) {
    Rig r; fe::ShellBatchPlasticityBinding oracle;
    ASSERT_TRUE(InitializeCollection(r,oracle,!legacy_triangle,legacy_triangle));
    ASSERT_TRUE(AssembleCollectionForBinding(r));
    EXPECT_EQ(r.publication.Initialize(r.owner,r.qeph,r.t3).status,fe::ShellPublicationStatus::InvalidInput);
    EXPECT_EQ(r.owner.accepted().epoch,0u);
  }
}
TEST_F(MixedShellCuda, CollectionByteCapAndDifferentInventoryFailBeforePublication) {
  Rig r; fe::ShellBatchPlasticityBinding oracle; ASSERT_TRUE(InitializeCollection(r,oracle));
  q::QephBatchConfig config; config.owner=r.owner.accepted(); config.element_count=1;
  config.configuration_id=Configuration; config.qualification_id=Qualification; config.usage=q::BatchUsage::PrescribedFields;
  config.max_device_bytes=active_shell_test::QBytes(1,Nodes);
  q::QephBatch capped;
  EXPECT_EQ(capped.InitializeJoined(config,r.binding,oracle).status,q::BatchStatus::ResourceLimit);
  EXPECT_EQ(capped.allocations().device_allocations,0u);
  plasticity_binding_test::Fixture f; f.Translate(.125); fe::ShellBatchBinding other;
  ASSERT_EQ(other.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  q::QephBatch mismatched;
  EXPECT_EQ(mismatched.InitializeJoined(config,other,oracle).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(mismatched.allocations().device_allocations,0u);
}
} // namespace resident_plasticity_test
