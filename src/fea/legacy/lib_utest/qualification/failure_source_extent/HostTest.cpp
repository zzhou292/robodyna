// SPDX-License-Identifier: MIT
#include "Support.h"
namespace failure_source_extent_test {
TEST(FailureSourceExtentHost, CompleteMixedFamilyMapsMatchEveryPrefixBoundary) {
  shell_output_range_test::Collection fixture(3);ASSERT_TRUE(fixture.ready());
  const Family families[]{Family::Qeph,Family::T3,Family::Qbat};
  const std::size_t sizes[]{6,9,3};
  for(unsigned family=0;family<3;++family) {
    const auto n=sizes[family];
    for(auto count:{std::size_t(0),std::size_t(1),n-1,n,n+1,SIZE_MAX}) {
      SCOPED_TRACE(::testing::Message()<<family<<":"<<count);
      Compare(fixture.failure,families[family],count,count<=n);
    }
  }
  for(auto family:{Family::None,static_cast<Family>(255)}) {
    Compare(fixture.failure,family,0,true);Compare(fixture.failure,family,1,false);
  }
}
TEST(FailureSourceExtentHost, MissingAndAbsentFamiliesKeepEmptyAndShapeErrorContracts) {
  fe::ShellBatchFailureBinding empty;
  for(auto family:{Family::Qeph,Family::T3,Family::Qbat,Family::None,static_cast<Family>(255)}) {
    Compare(empty,family,0,true);Compare(empty,family,1,false);Compare(empty,family,SIZE_MAX,false);
  }
  resident_failure_test::Source source;fe::ShellBatchBinding shells;fe::ShellBatchPlasticityBinding catalog;
  ASSERT_TRUE(source.Prepare(shells,catalog));fe::ShellBatchFailureBinding binding;
  ASSERT_EQ(binding.Initialize(catalog,source.failures.data(),source.failures.size()).status,fe::ShellPlasticityBindingStatus::Success);
  Compare(binding,Family::Qbat,0,true);Compare(binding,Family::Qbat,1,false);
  for(auto count:{std::size_t(0),std::size_t(1),std::size_t(2),SIZE_MAX}) {
    Compare(empty,Family::None,count,false,false);
    Compare(empty,Family::None,count,false,true,2);
    Compare(binding,Family::Qeph,count,false,true,0,count==2?1:2);
  }
  Compare(binding,Family::Qeph,2,true,true,1);
}
TEST(FailureSourceExtentHost, CopiesMovesAndOwnedLifetimePreserveAllFamilyMaps) {
  fe::ShellBatchFailureBinding held;
  const fe::ShellFailureParentInput* tail=nullptr;
  {
    shell_output_range_test::Collection fixture(2);ASSERT_TRUE(fixture.ready());
    auto copy=fixture.failure;fe::ShellBatchFailureBinding moved(std::move(copy));
    Compare(copy,Family::Qeph,1,false);Compare(copy,Family::None,0,true);
    tail=moved.parent(Family::T3,5);ASSERT_NE(tail,nullptr);held=std::move(moved);
    fixture.failures.clear();fixture.parents.clear();
    Compare(held,Family::Qeph,4,true);Compare(held,Family::T3,6,true);Compare(held,Family::Qbat,2,true);
  }
  EXPECT_EQ(held.parent(Family::T3,5),tail);
  Compare(held,Family::Qeph,4,true);Compare(held,Family::T3,6,true);Compare(held,Family::Qbat,2,true);
  Compare(held,Family::Qeph,5,false);Compare(held,Family::T3,7,false);Compare(held,Family::Qbat,3,false);
}
TEST(FailureSourceExtentHost, IncompleteOrReorderedInitializationCannotCreateAnExtent) {
  shell_output_range_test::Collection fixture;ASSERT_TRUE(fixture.ready());
  for(unsigned fault=0;fault<4;++fault) {
    auto rows=fixture.failures;auto count=rows.size();
    if(fault==0)--count;
    else if(fault==1)rows.back()=rows.front();
    else if(fault==2)std::swap(rows.front(),rows.back());
    else rows.back().source.family=Family::None;
    fe::ShellBatchFailureBinding rejected;
    EXPECT_NE(rejected.InitializeExecution(fixture.catalog,rows.data(),count).status,fe::ShellPlasticityBindingStatus::Success);
    EXPECT_FALSE(rejected.prepared());Compare(rejected,Family::Qeph,1,false);
    ASSERT_EQ(rejected.InitializeExecution(fixture.catalog,fixture.failures.data(),fixture.failures.size()).status,fe::ShellPlasticityBindingStatus::Success);
    Compare(rejected,Family::Qeph,2,true);Compare(rejected,Family::T3,3,true);Compare(rejected,Family::Qbat,1,true);
  }
}
} // namespace failure_source_extent_test
