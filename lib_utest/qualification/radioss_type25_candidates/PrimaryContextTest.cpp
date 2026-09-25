#include "PrimaryContextFixture.h"
#include "ObservedPrimaryContext.h"
#include <gtest/gtest.h>
using namespace candidate_test;
TEST(NativePrimaryContext,AllWorkerThresholdsAndUnshiftedRoleClassesMatchCanonicalPrimary) {
  const auto cases=PrimaryCases();RecordProperty("primary_context_rows",static_cast<int>(cases.size()));
  for(std::size_t i=0;i<cases.size();++i) {
    SCOPED_TRACE(i);const auto& cse=cases[i];c::FilterResult result;
    ASSERT_EQ(c::EvaluatePacked(cse.row,&result),c::Status::Ok);
    ASSERT_EQ(Bits(result.squared_clearance),Bits(NativeThreshold(cse.row,cse.primary,cse.row.segment_type)));
    for(int worker=1;worker<=cse.primary;++worker)for(int unshifted:{0,cse.primary+1,3*cse.primary+1}) {
      const double native=NativeThreshold(cse.row,worker,unshifted);
      ASSERT_EQ(Bits(result.squared_clearance),Bits(native));ASSERT_EQ(result.included,native!=0.);
    }
  }
}
TEST(NativePrimaryContext,FullClassificationCountChangesOrdinaryShellMembership) {
  auto row=PrimaryCases().front().row;row.nodes[3]=4;row.vertices[2]={1,1,0};row.vertices[3]={-1,1,0};
  row.secondary={0,0,0};row.segment_type=2;row.symmetry=1;row.main_count=1;
  EXPECT_EQ(NativeThreshold(row,1,2),0.);EXPECT_GT(NativeThreshold(row,2,2),0.);
  c::FilterResult canonical;ASSERT_EQ(c::EvaluatePacked(row,&canonical),c::Status::Ok);EXPECT_FALSE(canonical.included);
  row.main_count=2;ASSERT_EQ(c::EvaluatePacked(row,&canonical),c::Status::Ok);EXPECT_TRUE(canonical.included);
}
TEST(NativePrimaryContext,ArbitraryOrNegativePrefixDoesNotHaveUnshiftedRoleInvariance) {
  auto row=PrimaryCases().front().row;row.symmetry=1;row.secondary={0,0,0};
  EXPECT_GT(NativeThreshold(row,2,1),0.);EXPECT_EQ(NativeThreshold(row,4,5),0.);
  EXPECT_GT(NativeThreshold(row,2,-1),0.);EXPECT_EQ(NativeThreshold(row,4,0),0.);
}

TEST(NativePrimaryContext,ActualInitialChunksMatchCanonicalActualRoleWithoutWorkerScheduling) {
  const auto rows=ObservedPrimaryRows();ASSERT_EQ(rows.size(),120u);unsigned admitted=0;
  for(const auto& observed:rows) {
    bool screen=false;ASSERT_EQ(c::EvaluateScreen(observed.row.screen,&screen),c::Status::Ok);
    ASSERT_EQ(screen,NativeScreen(observed.row.screen));
    c::FilterResult output;ASSERT_EQ(c::EvaluateLocal(observed.row,&output),c::Status::Ok);
    if(!screen){EXPECT_FALSE(output.included);continue;}
    c::PackedRow canonical;ASSERT_EQ(c::PackLocal(observed.row,&canonical),c::Status::Ok);
    const auto packing=NativePack(observed.row);ASSERT_EQ(Bits(canonical.gap),Bits(packing.gap));
    ASSERT_EQ(canonical.symmetry,packing.symmetry);canonical.gap=packing.gap;canonical.symmetry=packing.symmetry;
    const double native=NativeThreshold(canonical,observed.worker_count,observed.unshifted_role);
    EXPECT_EQ(Bits(output.squared_clearance),Bits(native));EXPECT_EQ(output.included,native!=0.);
    admitted+=output.included;
  }
  EXPECT_GT(admitted,0u);RecordProperty("observed_native_initial_candidates",admitted);
}

TEST(NativePrimaryContext,RemainderAndEmptyWorkersCoverOnlyAuthenticPrimaryRows) {
  auto row=PrimaryCases().front().row;row.symmetry=1;row.secondary={0,0,0};unsigned empty=0;
  for(int primary:{1,3,5,8})for(int workers:{1,2,4,11}) {
    std::vector<int> roles(primary),seen(primary,0);
    for(int i=0;i<primary;++i)roles[i]=i%3==0?0:(i%3==1?primary+i+1:3*primary+i+1);
    for(int worker=0;worker<workers;++worker) {
      const int first=worker*primary/workers,last=(worker+1)*primary/workers,count=last-first;
      if(!count){++empty;continue;}
      for(int local=0;local<count;++local) {
        ++seen[first+local];row.main_count=primary;row.segment_type=roles[first+local];c::FilterResult out;
        ASSERT_EQ(c::EvaluatePacked(row,&out),c::Status::Ok);
        EXPECT_EQ(Bits(out.squared_clearance),Bits(NativeThreshold(row,count,roles[local])));
      }
    }
    for(auto count:seen)EXPECT_EQ(count,1);
  }
  EXPECT_GT(empty,0u);
}
