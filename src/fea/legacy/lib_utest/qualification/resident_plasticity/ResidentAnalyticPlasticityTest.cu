#include "ResidentCollectionFixture.h"
#include "../active_shell_collection/StorageExpectations.h"

namespace resident_plasticity_test {
namespace {
__global__ void CollapseAnalyticTriangle(fe::NodalPreparedView view) {
  for(unsigned c=0;c<3;++c)
    const_cast<double*>(view.kinematics.position_xyz)[12+c]=view.kinematics.position_xyz[3+c];
}
}
TEST_F(MixedShellCuda, AnalyticMixedAndZeroCurveCollectionsYieldRollbackAndRetryExactly) {
  for(bool all:{false,true}) {
    SCOPED_TRACE(all); Rig r; fe::ShellBatchPlasticityBinding oracle;
    ASSERT_TRUE(InitializeAnalyticCollection(r,oracle,all)); ASSERT_TRUE(r.Bind());
    EXPECT_EQ(oracle.curve_point_count(),all?0u:3u);
    const auto qa=r.qeph.allocations(),ta=r.t3.allocations();
    fe::shell_batch_plasticity_detail::Layout layout;
    ASSERT_TRUE(layout.Initialize(1,oracle.curve_point_count(),fe::MaxShellResidentDeviceBytes));
    EXPECT_EQ(qa.device_bytes,active_shell_test::QBytes(1,Nodes)+layout.bytes);
    EXPECT_EQ(ta.device_bytes,active_shell_test::TBytes(1,Nodes)+layout.bytes);
    EXPECT_EQ(qa.device_allocations,2u); EXPECT_EQ(ta.device_allocations,2u);
    Staged old_shell; SectionPair old_section;
    ASSERT_TRUE(Accepted(r,old_shell)); ASSERT_TRUE(Sections(r,old_section));
    for(unsigned step=0;step<3;++step) {
      SCOPED_TRACE(step);
      Prepared prepared; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,step),prepared));
      Staged candidate; ASSERT_TRUE(Evaluate(r,prepared,candidate));
      SectionPair sections; ASSERT_TRUE(Sections(r,sections,&candidate));
      CheckHostAdapters(r,prepared,old_shell,old_section,candidate,sections,oracle);
      ASSERT_GT(sections.q.diagnostics.maximum_plastic_strain,0);
      ASSERT_GT(sections.t.diagnostics.maximum_plastic_strain,0);
      // Schedule step 1 is a hold. At this fixture's millisecond step the
      // native 10 kHz filter has alpha=1, so zero total rate clears its history.
      // Other steps load/reverse; the mixed QEPH declaration disables rate.
      if(step==1) {
        ASSERT_EQ(sections.t.history.point[0].filtered_rate_per_s,0);
        ASSERT_EQ(sections.q.history.point[0].filtered_rate_per_s,0);
      } else {
        ASSERT_GT(sections.t.history.point[0].filtered_rate_per_s,0);
        if(all) ASSERT_GT(sections.q.history.point[0].filtered_rate_per_s,0);
        else ASSERT_EQ(sections.q.history.point[0].filtered_rate_per_s,0);
      }
      ASSERT_TRUE(Publish(r,prepared,candidate)); old_shell=candidate; old_section=sections;
    }
    Snapshot before; ASSERT_TRUE(Read(r.owner,before));
    Prepared clean_p; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,3),clean_p));
    Staged clean; ASSERT_TRUE(Evaluate(r,clean_p,clean)); SectionPair clean_section;
    ASSERT_TRUE(Sections(r,clean_section,&clean)); r.Discard();
    Prepared bad; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,3),bad)); q::BatchDiagnostics qd;
    ASSERT_EQ(r.qeph.EvaluateCandidate(bad.view,&qd).status,q::BatchStatus::Success);
    CollapseAnalyticTriangle<<<1,1,0,bad.view.stream>>>(bad.view); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    t::BatchDiagnostics td; EXPECT_EQ(r.t3.EvaluateCandidate(bad.view,&td).status,t::BatchStatus::ElementFailure);
    r.Discard(); Snapshot unchanged; Staged shells; SectionPair sections;
    ASSERT_TRUE(Read(r.owner,unchanged)); SameState(unchanged,before);
    ASSERT_TRUE(Accepted(r,shells)); ExactResults(shells,old_shell);
    ASSERT_TRUE(Sections(r,sections)); SameSections(sections,old_section);
    Prepared retry_p; ASSERT_TRUE(Prepare(r,PlasticSchedule(r,3),retry_p));
    Staged retry; ASSERT_TRUE(Evaluate(r,retry_p,retry)); ExactResults(retry,clean);
    ASSERT_TRUE(Sections(r,sections,&retry)); SameSections(sections,clean_section);
    ASSERT_TRUE(Publish(r,retry_p,retry));
    EXPECT_EQ(r.qeph.allocations().device_bytes,qa.device_bytes); EXPECT_EQ(r.t3.allocations().device_bytes,ta.device_bytes);
    EXPECT_EQ(r.qeph.allocations().device_allocations,qa.device_allocations);
    EXPECT_EQ(r.t3.allocations().device_allocations,ta.device_allocations);
  }
}
} // namespace resident_plasticity_test
