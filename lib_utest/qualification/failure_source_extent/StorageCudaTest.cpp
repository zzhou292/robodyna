// SPDX-License-Identifier: MIT
#include "Support.h"
namespace failure_source_extent_test {
TEST(FailureSourceExtentCuda, ActualStorageInitializationChecksBothSlabsAndRetainsOwnedSource) {
  int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
  for(auto family:{Family::Qeph,Family::T3})for(std::size_t count:{1u,2u}) {
    storage::FailureHostStorage actual;storage::FailureLayout layout;
    ASSERT_TRUE(layout.Initialize(count,1u<<20));
    EXPECT_EQ(actual.CheckActivitySources(0,count).status,Status::InvalidInput);
    {
      resident_failure_test::Source source;fe::ShellBatchBinding shells;fe::ShellBatchPlasticityBinding catalog;
      ASSERT_TRUE(source.Prepare(shells,catalog));fe::ShellBatchFailureBinding binding;
      ASSERT_EQ(binding.Initialize(catalog,source.failures.data(),source.failures.size()).status,fe::ShellPlasticityBindingStatus::Success);
      ASSERT_EQ(actual.Initialize(binding,family,count,layout).status,Status::Success);
    }
    for(unsigned slab:{0u,1u}) {
      const auto result=actual.CheckActivitySources(slab,count);EXPECT_EQ(result.status,Status::Success);EXPECT_STREQ(result.message,"OK");
      const auto old=reference::FrozenCheck(true,actual.binding(),family,count,slab,count);
      EXPECT_EQ(result.status,old.status);EXPECT_STREQ(result.message,old.message);
      EXPECT_EQ(actual.CheckReadSources(slab,count,nullptr).status,Status::InvalidInput);
      fe::ShellBatchLayeredSection nonnull;
      EXPECT_EQ(actual.CheckReadSources(slab,count,&nonnull).status,Status::Success);
    }
    for(auto bad_count:{std::size_t(0),count+1,SIZE_MAX}) {
      const auto result=actual.CheckActivitySources(0,bad_count);
      EXPECT_EQ(result.status,Status::InvalidInput);EXPECT_STREQ(result.message,"Invalid failure readback shape");
    }
    EXPECT_EQ(actual.CheckActivitySources(2,count).status,Status::InvalidInput);
    EXPECT_GT(actual.device_bytes(),0u);
  }
}
TEST(FailureSourceExtentCuda, RejectedFamilyExtentAndLayoutLeaveStorageReusable) {
  int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
  resident_failure_test::Source source;fe::ShellBatchBinding shells;fe::ShellBatchPlasticityBinding catalog;
  ASSERT_TRUE(source.Prepare(shells,catalog));fe::ShellBatchFailureBinding binding;
  ASSERT_EQ(binding.Initialize(catalog,source.failures.data(),source.failures.size()).status,fe::ShellPlasticityBindingStatus::Success);
  storage::FailureLayout good,too_long;ASSERT_TRUE(good.Initialize(2,1u<<20));ASSERT_TRUE(too_long.Initialize(3,1u<<20));
  storage::FailureHostStorage actual;fe::ShellBatchFailureBinding empty;
  EXPECT_EQ(actual.Initialize(empty,Family::Qeph,2,good).status,Status::InvalidInput);
  for(auto family:{Family::None,Family::Qbat,static_cast<Family>(255)})
    EXPECT_EQ(actual.Initialize(binding,family,2,good).status,Status::InvalidInput);
  EXPECT_EQ(actual.Initialize(binding,Family::Qeph,1,good).status,Status::InvalidInput);
  const auto missing=actual.Initialize(binding,Family::Qeph,3,too_long);
  EXPECT_EQ(missing.status,Status::InvalidInput);EXPECT_STREQ(missing.message,"Failure sidecar family source is incomplete");
  EXPECT_EQ(actual.device(),nullptr);EXPECT_EQ(actual.device_bytes(),0u);
  ASSERT_EQ(actual.Initialize(binding,Family::Qeph,2,good).status,Status::Success);
  EXPECT_EQ(actual.CheckActivitySources(0,2).status,Status::Success);
}
} // namespace failure_source_extent_test
