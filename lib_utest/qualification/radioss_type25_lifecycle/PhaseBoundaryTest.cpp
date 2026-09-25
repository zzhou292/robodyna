// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "NativeOracle.h"
#include <array>
#include <limits>
namespace type25_lifecycle_test {
namespace {
l::HostResult Complete(Fixture& f,const l::OptimizedRow& optimized,
    const n::units_detail::Factors& units) {
  std::array<l::Occurrence,128> occurrences{};
  std::array<n::NativeRawGeometryResult,128> geometry{};
  std::array<int,256> sliding{};
  l::RowScratch scratch{occurrences.data(),geometry.data(),occurrences.size(),
                       sliding.data(),sliding.size(),0};
  const auto prepared=l::detail::PrepareRowAfterNormals(f.Input(),0,scratch,units,optimized);
  EXPECT_EQ(prepared.stage.report.status,n::selection::Status::Ok);
  EXPECT_TRUE(prepared.stage.report.count_complete);
  const auto complete=l::detail::CompleteRow(f.Input(),0,scratch,units,prepared);
  EXPECT_EQ(complete.report.status,n::selection::Status::Ok);
  l::HostResult result;result.rows.push_back(complete.value);
  result.occurrences.assign(occurrences.begin(),occurrences.begin()+complete.occurrence_count);
  result.geometry.assign(geometry.begin(),geometry.begin()+complete.occurrence_count);
  return result;
}
}
TEST(Type25LifecycleBoundary, NormalValuesAreFirstReadAfterTheExplicitBoundary) {
  for(bool retained:{false,true}) {
    Fixture f;if(retained){f.Retained();f.positions[18]=5;}
    const auto mains=f.mains;const auto normals=f.normals;
    const auto before=f.accepted;
    n::units_detail::Factors units;ASSERT_TRUE(l::detail::Factors(f.Input().current,units));
    for(auto& main:f.mains)for(auto& normal:main.normal_slot)
      normal={std::numeric_limits<float>::quiet_NaN(),0,0};
    for(auto& reference:f.normals)for(auto& normal:reference.bisector)
      normal={std::numeric_limits<float>::quiet_NaN(),0,0};
    const auto optimized=l::detail::PrepareRowBeforeNormals(f.Input(),0,units);
    ASSERT_EQ(optimized.stage.report.status,n::selection::Status::Ok);
    EXPECT_EQ(optimized.stage.report.stage,l::Stage::Optimize);
    EXPECT_FALSE(optimized.stage.report.count_complete);
    EXPECT_EQ(optimized.stage.report.required_candidates,0u);
    EXPECT_EQ(optimized.stage.value.sliding_count,0u);
    // A real NORMP implementation is a separate gate. This replaces only its
    // declared output fields and proves the lifecycle no longer reads them early.
    f.mains=mains;f.normals=normals;
    Same(Complete(f,optimized,units),OracleLifecycle(f.Input()));
    type25_geometry_test::Same(f.accepted[0],before[0],true);
  }
}
TEST(Type25LifecycleBoundary, DeletedMainSnapshotSurvivesTheNormalBarrier) {
  Fixture f;f.Retained();f.mains[0].coefficient=0;f.positions[18]=5;
  n::units_detail::Factors units;ASSERT_TRUE(l::detail::Factors(f.Input().current,units));
  const auto optimized=l::detail::PrepareRowBeforeNormals(f.Input(),0,units);
  ASSERT_EQ(optimized.stage.report.status,n::selection::Status::Ok);
  EXPECT_EQ(optimized.optimization_leave,-1);
  EXPECT_EQ(optimized.stage.value.history.row.irtlm[2],0);
  EXPECT_EQ(optimized.stage.value.optimized_count,0u);
  Same(Complete(f,optimized,units),OracleLifecycle(f.Input()));
}
TEST(Type25LifecycleBoundary, EarlyFailureDoesNotReadNormalOrTouchLaterScratch) {
  Fixture f;f.Retained();f.accepted[0].row.irtlm[2]=0;
  n::units_detail::Factors units;ASSERT_TRUE(l::detail::Factors(f.Input().current,units));
  const auto optimized=l::detail::PrepareRowBeforeNormals(f.Input(),0,units);
  ASSERT_EQ(optimized.stage.report.status,n::selection::Status::InvalidInput);
  EXPECT_EQ(optimized.stage.report.stage,l::Stage::Begin);
  // No scratch storage or normal fields are needed after a rejected Begin.
  auto input=f.Input();input.source.normals=nullptr;input.source.mains=nullptr;
  const auto prepared=l::detail::PrepareRowAfterNormals(input,0,{},units,optimized);
  EXPECT_EQ(prepared.stage.report.status,n::selection::Status::InvalidInput);
  EXPECT_EQ(prepared.stage.report.stage,l::Stage::Begin);
  EXPECT_FALSE(prepared.stage.report.count_complete);
}
} // namespace type25_lifecycle_test
