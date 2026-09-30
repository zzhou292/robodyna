#include "WallRawReportTestFixture.h"

namespace tl::qualification::qeph::wall_recurrence {
namespace rt=raw_test;
TEST(QephWallRawCapture, FrozenCellIntervalCountsAndInterruptedModelRemainDurable) {
  EXPECT_EQ(PlannedNativeCellIntervals(1),4350u); EXPECT_EQ(PlannedNativeCellIntervals(2),14964u);
  EXPECT_EQ(3*(PlannedNativeCellIntervals(1)+PlannedNativeCellIntervals(2)),57942u);
  EXPECT_THROW(PlannedNativeCellIntervals(3),std::invalid_argument);
  rt::Directory directory; const auto path=directory.path/"raw";
  RawJobWriter writer(path,1,0,rt::Provenance(),"synthetic.json",ScreenSetByteCap);
  struct StopAfterModel {};
  unsigned callbacks=0;
  EXPECT_THROW(CollectRawJob(1,0,[&](const RawJob& job,RawProgress event) {
    EXPECT_EQ(event.kind,RawProgressKind::Model); EXPECT_TRUE(job.model.prepared());
    EXPECT_FALSE(job.collection_complete); ++callbacks; writer(job,event); throw StopAfterModel{};
  }),StopAfterModel);
  EXPECT_EQ(callbacks,1u); EXPECT_FALSE(writer.receipt().final_index_present);
  EXPECT_FALSE(std::filesystem::exists(path/"index.json")); rt::CheckFiles(path,writer.receipt());
  const auto progress=rt::Read(path/"progress-001.json");
  EXPECT_FALSE(progress["collection_complete"].GetBool()); EXPECT_TRUE(progress["model_prepared"].GetBool());
  EXPECT_FALSE(progress["native_cell_interval_count_exact"].GetBool());
  EXPECT_EQ(progress["files"].Size(),3u); // Provenance, initial inventory, model.
}
TEST(QephWallRawCapture, CompleteMeansCollectedRawDataNotPassedNumericalChecks) {
  RawStep step; constexpr unsigned cells=1,nodes=4,dimension=109;
  for(unsigned a=0;a<3;++a) {
    step.native_attempted[a]=true; step.native[a].baseline_complete=true;
    step.native[a].derivative.complete=true; step.native[a].derivative.completed_columns=dimension;
  }
  for(unsigned b=0;b<2;++b) {
    step.contact_attempted[b]=true; auto& p=step.contact[b]; p.baseline_complete=true;
    p.directions.resize(nodes+7);
    for(auto& d:p.directions) d.completed_samples=3;
    p.passed=false; p.complete=false; // Raw collection is independent of local quotient verdicts.
  }
  EXPECT_TRUE(CompleteRawStep(step,cells));
  step.contact[1].directions.back().completed_samples=2;
  EXPECT_FALSE(CompleteRawStep(step,cells));
}
} // namespace tl::qualification::qeph::wall_recurrence
