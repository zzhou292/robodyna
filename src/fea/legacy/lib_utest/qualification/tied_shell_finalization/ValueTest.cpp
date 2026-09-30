#include "Fixture.h"
#include "lib_src/constraints/tied_shell/search/FinalizationInternal.h"
#include <limits>
namespace tied_finalization_test {
TEST(TiedFinalizationValues, StableMapsKeepDeclaredMasterRanksAndUnclampedWarningCoordinates) {
  Fixture f;
  f.choices[1]={};
  f.choices[2].projection.s=1.6;
  f.choices[3].projection.s=1.1;
  ts::FinalizedSearch out;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out));
  const auto& d=*out.data();
  EXPECT_EQ(d.slaves,(std::vector<std::uint32_t>{0,3}));
  EXPECT_EQ(d.slave_inverse,(std::vector<std::uint32_t>{0,UINT32_MAX,UINT32_MAX,1}));
  EXPECT_EQ(d.main_nodes,(std::vector<std::uint32_t>{0,2,3,5,6}));
  EXPECT_EQ(d.selected_masters,(std::vector<std::uint64_t>{1,2}));
  EXPECT_EQ(d.dispositions[1],ts::FinalizationDisposition::Unmatched);
  EXPECT_EQ(d.dispositions[2],ts::FinalizationDisposition::OutsideParameters);
  EXPECT_EQ(d.dispositions[3],ts::FinalizationDisposition::Kept);
  EXPECT_EQ(d.st[1][0],1.1);
  ASSERT_EQ(d.messages.size(),10u);
  EXPECT_EQ(d.messages[0].id,1071u);
  EXPECT_EQ(d.messages[1].id,1158u);
  EXPECT_EQ(d.messages[2].id,1079u);
  EXPECT_EQ(d.messages[3].action,ts::NativeMessageAction::Flush);
  EXPECT_EQ(d.messages[4].id,1078u);
  EXPECT_EQ(d.messages[4].severity,ts::NativeMessageSeverity::Error);
  EXPECT_EQ(d.cleared_dmin_entries,4u);
}
TEST(TiedFinalizationValues, ExactBoundariesReachNativeDisposition) {
  Fixture f;
  f.choices[0].projection.s=1.02;
  f.choices[1].projection.s=1.5;
  f.choices[2].projection.t=-1.5;
  f.choices[3].projection.t=std::nextafter(-1.5,-2.);
  ts::FinalizedSearch out;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out));
  EXPECT_EQ(out.data()->slaves,(std::vector<std::uint32_t>{0,1,2}));
  EXPECT_EQ(out.data()->messages[0].id,1079u);
  EXPECT_EQ(out.data()->messages[0].original_slave,1u);
  EXPECT_EQ(out.data()->messages[2].id,1158u);
}
TEST(TiedFinalizationValues, AllUnmatchedPublishesEmptyMapsAndDistinctDirectWarning) {
  Fixture f;
  for(auto& c:f.choices) c={};
  ts::FinalizedSearch out;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out));
  EXPECT_TRUE(out.data()->slaves.empty());
  EXPECT_TRUE(out.data()->main_nodes.empty());
  ASSERT_EQ(out.data()->messages.size(),12u);
  EXPECT_EQ(out.data()->messages[4].action,ts::NativeMessageAction::Flush);
  EXPECT_EQ(out.data()->messages[5].id,1217u);
  EXPECT_EQ(out.data()->messages[5].action,ts::NativeMessageAction::Direct);
}
TEST(TiedFinalizationValues, LastInvalidRankAndNonfinitePreservePublishedHandleAndRetry) {
  Fixture f;
  ts::FinalizedSearch out;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out));
  const auto prior=out;
  f.choices.back().ordered_master=3;
  EXPECT_EQ(ts::FinalizeSearch(f.Input(),&out).status,ts::FinalizationStatus::InvalidInput);
  EXPECT_EQ(out.data(),prior.data());
  f.choices.back().ordered_master=2;
  f.choices.back().projection.t=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ts::FinalizeSearch(f.Input(),&out));
  EXPECT_EQ(out.data(),prior.data());
  f.choices.back().projection.t=-.2*3;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out));
  EXPECT_EQ(out.data()->st,prior.data()->st);
  EXPECT_EQ(out.data()->main_nodes,prior.data()->main_nodes);
}
TEST(TiedFinalizationValues, BudgetsRejectBeforeBorrowedReadsAndIncludeOldBacking) {
  Fixture f;
  ts::FinalizedSearch out;
  std::size_t bytes=0;
  ASSERT_TRUE(ts::finalization_detail::Preflight(f.Input(),out,{},bytes));
  auto limits=ts::FinalizationLimits{};
  limits.max_host_bytes=bytes;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out,limits));
  std::size_t replacement=0;
  ASSERT_TRUE(ts::finalization_detail::Preflight(f.Input(),out,{},replacement));
  EXPECT_EQ(replacement,bytes+out.data()->owned_payload_bytes);
  const auto prior=out;
  auto poisoned=f.Input();
  poisoned.choices=reinterpret_cast<const ts::SearchChoice*>(alignof(ts::SearchChoice));
  limits.max_host_bytes=replacement-1;
  EXPECT_EQ(ts::FinalizeSearch(poisoned,&out,limits).status,ts::FinalizationStatus::ResourceLimit);
  EXPECT_EQ(out.data(),prior.data());
  limits.max_host_bytes++;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out,limits));
}
TEST(TiedFinalizationValues, DuplicateNSVAndMissingTailMSRRejectOriginalProfile) {
  Fixture f;
  ts::FinalizedSearch out;
  f.slaves.back()=f.slaves.front();
  EXPECT_FALSE(ts::FinalizeSearch(f.Input(),&out));
  EXPECT_FALSE(out.data());
  f.slaves.back()=7;
  f.masters.back()[3]=7;
  EXPECT_FALSE(ts::FinalizeSearch(f.Input(),&out));
  f.masters.back()[3]=4;
  ASSERT_TRUE(ts::FinalizeSearch(f.Input(),&out));
}
}
