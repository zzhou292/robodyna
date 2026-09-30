// SPDX-License-Identifier: AGPL-3.0-or-later
#include "HistoryFixture.h"
#include "Assertions.h"
#include "NativeOracle.h"
namespace type25_geometry_test {
namespace {
void Compare(double time,HistoryFixture& f) {
  const auto expected=OracleHistory(Profile(),time,f.geometry,f.selected);
  ASSERT_EQ(n::FinalizeNativeGeometryHistory(Profile(),time,f.View()).status,n::GeometryStatus::Ok);
  ASSERT_EQ(f.staged.size(),expected.rows.size());ASSERT_EQ(f.results.size(),expected.results.size());
  for(std::size_t i=0;i<f.staged.size();++i)Same(f.staged[i],expected.rows[i]);
  for(std::size_t i=0;i<f.results.size();++i) {
    Same(f.results[i].geometry,expected.results[i].geometry,true);
    Number(f.results[i].penetration,expected.results[i].penetration,true);
  }
}
void Preserved(const HistoryFixture& f,const std::vector<n::NativeGeometryHistory>& rows,
               const std::vector<n::NativeGeometryFinalResult>& results) {
  ASSERT_EQ(f.staged.size(),rows.size());ASSERT_EQ(f.results.size(),results.size());
  for(std::size_t i=0;i<rows.size();++i)Same(f.staged[i],rows[i]);
  for(std::size_t i=0;i<results.size();++i) {
    Same(f.results[i].geometry,results[i].geometry,true);
    Number(f.results[i].penetration,results[i].penetration,true);
  }
}
}
TEST(Type25GeometryHistory, CompleteInitialOffsetPassAndZeroPenetrationStiffnessMatchNative) {
  auto row=History(Quad());
  HistoryFixture f({Raw(.1,500),Raw(.3,600),Raw(.2,550)},{row});
  Compare(0,f);
  EXPECT_DOUBLE_EQ(f.staged[0].row.penetration_offset,.3);
  EXPECT_DOUBLE_EQ(f.staged[0].row.history.normal.staged_stiffness,600);
  for(const auto& r:f.results)EXPECT_DOUBLE_EQ(r.penetration,0);
  EXPECT_DOUBLE_EQ(f.selected[0].row.penetration_offset,.05);
}
TEST(Type25GeometryHistory, NewImpactRepeatsKeepNativeOrdinalLastOffsetAndAllOtherState) {
  for(bool reverse:{false,true}) {
    auto row=History(Quad());row.row.irtlm[0]=-3;
    std::vector<n::NativeRawGeometryResult> raw{Raw(.1),Raw(.3),Raw(.2,0)};
    if(reverse)std::reverse(raw.begin(),raw.end());
    HistoryFixture f(raw,{row});Compare(1,f);
    EXPECT_DOUBLE_EQ(f.staged[0].row.penetration_offset,raw.back().geometric_penetration);
    for(const auto& r:f.results)EXPECT_DOUBLE_EQ(r.penetration,0);
  }
}
TEST(Type25GeometryHistory, LaterCohortConsumesPriorOffsetWithoutAnotherInitialization) {
  HistoryFixture first({Raw(.1)},{History(Quad())});Compare(0,first);
  HistoryFixture next({Raw(.3)},first.staged);Compare(1,next);
  EXPECT_DOUBLE_EQ(next.results[0].penetration,.3-.1);
  EXPECT_DOUBLE_EQ(next.staged[0].row.penetration_offset,.1);
}
TEST(Type25GeometryHistory, IndependentRowsAndEmptyCohortPreserveDefinedHistory) {
  auto a=History(Quad()),b=a;b.secondary_source_id=2202;
  auto x=Raw(-0.0),y=Raw(.2);y.key.secondary_source_id=2202;y.key.history_index=1;
  HistoryFixture f({x,y},{a,b});Compare(1,f);
  EXPECT_DOUBLE_EQ(f.staged[0].row.history.normal.staged_stiffness,400);
  HistoryFixture empty({},f.staged);Compare(1,empty);
}
TEST(Type25GeometryHistory, LateIdentityOrArithmeticFailurePublishesNothingAndRetryWorks) {
  for(unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);auto row=History(Quad());
    HistoryFixture f({Raw(.1),Raw(.2)},{row});
    const auto before_rows=f.staged;const auto before_results=f.results;
    auto& last=f.geometry.back();
    if(fault==0)last.key.history_index=1;
    if(fault==1)last.key.generation=8;
    if(fault==2)last.key.main_segment=4;
    if(fault==3)last.selection_code=2;
    if(fault==4)last.normal.x=std::numeric_limits<double>::quiet_NaN();
    if(fault==5)last.incoming_stiffness=-1;
    if(fault==6) {
      last.geometric_penetration=std::numeric_limits<double>::max();
      f.selected[0].row.penetration_offset=-std::numeric_limits<double>::max();
    }
    const auto report=n::FinalizeNativeGeometryHistory(Profile(),1,f.View());
    EXPECT_EQ(report.status,fault==6?n::GeometryStatus::NonfiniteResult:n::GeometryStatus::InvalidInput);
    EXPECT_EQ(report.geometry_index,1u);
    Preserved(f,before_rows,before_results);
    f.geometry.back()=Raw(.2);f.selected[0]=row;Compare(1,f);
  }
}
TEST(Type25GeometryHistory, ExactCapacityAliasedRangesAndOverflowFailBeforeAnyWrite) {
  HistoryFixture f({Raw(.1),Raw(.2)},{History(Quad())});
  const auto before_rows=f.staged;const auto before_results=f.results;
  const auto forecast=n::PreflightNativeGeometryHistory(2,1);
  ASSERT_EQ(forecast.status,n::GeometryStatus::Ok);
  auto limits=n::GeometryBatchLimits{};limits.scratch_bytes=forecast.total_scratch_bytes-1;
  EXPECT_EQ(n::FinalizeNativeGeometryHistory(Profile(),1,f.View(),limits).status,n::GeometryStatus::CapacityExceeded);
  Preserved(f,before_rows,before_results);
  auto view=f.View();view.scratch_results=view.results;
  EXPECT_EQ(n::FinalizeNativeGeometryHistory(Profile(),1,view).status,n::GeometryStatus::InvalidInput);
  Preserved(f,before_rows,before_results);
  view=f.View();view.staged_rows=const_cast<n::NativeGeometryHistory*>(view.selected_rows);
  EXPECT_EQ(n::FinalizeNativeGeometryHistory(Profile(),1,view).status,n::GeometryStatus::InvalidInput);
  Same(f.selected[0],History(Quad()));
  view=f.View();view.scratch_row_capacity=0;
  EXPECT_EQ(n::FinalizeNativeGeometryHistory(Profile(),1,view).status,n::GeometryStatus::CapacityExceeded);
  limits.scratch_bytes=forecast.total_scratch_bytes;
  ASSERT_EQ(n::FinalizeNativeGeometryHistory(Profile(),1,f.View(),limits).status,n::GeometryStatus::Ok);
  limits.rows=SIZE_MAX;
  EXPECT_EQ(n::PreflightNativeGeometryHistory(SIZE_MAX,1,limits).status,n::GeometryStatus::CapacityExceeded);
}

TEST(Type25GeometryHistory, UnreferencedInvalidHistoryRowRejectsEvenEmptyCohort) {
  for(bool empty:{false,true}) {
    auto a=History(Quad()),b=a;b.secondary_source_id=2202;
    b.row.history.normal.staged_stiffness=std::numeric_limits<double>::quiet_NaN();
    HistoryFixture f(empty?std::vector<n::NativeRawGeometryResult>{}:
                           std::vector<n::NativeRawGeometryResult>{Raw(.1)},{a,b});
    f.staged[1]=a;const auto before_rows=f.staged;const auto before_results=f.results;
    const auto report=n::FinalizeNativeGeometryHistory(Profile(),1,f.View());
    EXPECT_EQ(report.status,n::GeometryStatus::InvalidInput);EXPECT_EQ(report.history_index,1u);
    Preserved(f,before_rows,before_results);
  }
}

} // namespace type25_geometry_test
