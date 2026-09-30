#include "FailureResidentSource.h"
#include <limits>
namespace resident_failure_test {
using S=fe::ShellPlasticityBindingStatus;
TEST(ResidentFailureBinding,OwnsExactCompleteOrderedScopeAndRejectsUnsupportedPolicy) {
  Source source;fe::ShellBatchBinding native;fe::ShellBatchPlasticityBinding catalog;
  ASSERT_TRUE(source.Prepare(native,catalog));fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,source.failures.data(),4).status,S::Success);
  const auto original=source.failures[3];source.failures[3].constant.failure_strain=3;
  EXPECT_EQ(failure.parent(fe::ShellBindingFamily::T3,1)->constant.failure_strain,original.constant.failure_strain);
  EXPECT_EQ(failure.parent(0)->source.family,fe::ShellBindingFamily::T3);
  EXPECT_EQ(failure.parent(fe::ShellBindingFamily::Qeph,2),nullptr);
  EXPECT_TRUE(failure.Matches(catalog));fe::ShellBatchFailureBinding copy=failure;EXPECT_TRUE(copy.SameScope(failure));
  fe::ShellBatchFailureBinding other;ASSERT_EQ(other.Initialize(catalog,source.failures.data(),4).status,S::Success);
  EXPECT_FALSE(failure.SameScope(other));
  for(unsigned fault=0;fault<7;++fault) {
    auto input=source.failures;
    switch(fault){
      case 0:input[3].source.source_parent_id++;break;
      case 1:std::swap(input[2],input[3]);break;
      case 2:input[3].constant.failure_strain=std::numeric_limits<double>::quiet_NaN();break;
      case 3:input[3].policy=fe::ShellFailurePolicy::Tab1AnyPoint;break;
      case 4:input[0].policy=fe::ShellFailurePolicy::ConstantAllPoints;input[0].constant.failure_strain=1;break;
      case 5:input[0].constant.failure_strain=-0.;break;
      case 6:input[3].source.material_id++;break;
    }
    fe::ShellBatchFailureBinding rejected;EXPECT_NE(rejected.Initialize(catalog,input.data(),4).status,S::Success)<<fault;
    EXPECT_FALSE(rejected.prepared());EXPECT_EQ(rejected.parent(0),nullptr);
    EXPECT_EQ(rejected.Initialize(catalog,source.failures.data(),4).status,S::Success)<<fault;
    EXPECT_TRUE(rejected.SameScope(other));
  }
}
TEST(ResidentFailureBinding,CountAndByteCapsRejectBeforeBorrowedRowsAndPermitExactRetry) {
  Source source;fe::ShellBatchBinding native;fe::ShellBatchPlasticityBinding catalog;ASSERT_TRUE(source.Prepare(native,catalog));
  const auto* poison=reinterpret_cast<const fe::ShellFailureParentInput*>(std::uintptr_t(1));
  fe::ShellBatchFailureBinding rejected;fe::ShellBatchFailureLimits limits;limits.max_parents=3;
  EXPECT_EQ(rejected.Initialize(catalog,poison,4,limits).status,S::ResourceLimit);
  limits.max_parents=4;limits.max_host_bytes=1;
  EXPECT_EQ(rejected.Initialize(catalog,poison,4,limits).status,S::ResourceLimit);
  EXPECT_EQ(rejected.Initialize(catalog,poison,3).status,S::InvalidInput);
  EXPECT_FALSE(rejected.prepared());ASSERT_EQ(rejected.Initialize(catalog,source.failures.data(),4).status,S::Success);
  EXPECT_GT(rejected.host_bytes(),catalog.host_bytes());
}
} // namespace resident_failure_test
