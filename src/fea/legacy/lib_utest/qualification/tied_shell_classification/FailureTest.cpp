// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <climits>

namespace classification_test {
TEST(TiedClassificationFailure,CountsAndBytesPrecedePoisonedPayload) {
  Fixture f;
  ClassificationResult result;
  ASSERT_TRUE(Classify(f.Input(),&result));
  const auto saved=result;
  auto input=f.Input();
  input.context.nodes={reinterpret_cast<const ClassificationNode*>(1),SIZE_MAX};
  EXPECT_EQ(Classify(input,&result).status,ClassificationStatus::ResourceLimit);
  Same(saved,result);
  input=f.Input(); input.context.nodes.data=reinterpret_cast<const ClassificationNode*>(8);
  ClassificationLimits limits; limits.max_host_bytes=1;
  EXPECT_EQ(Classify(input,&result,limits).status,ClassificationStatus::ResourceLimit);
  Same(saved,result);
  input=f.Input(); input.interfaces.data=reinterpret_cast<const ClassificationInterface*>(8);
  EXPECT_EQ(Classify(input,&result,limits).status,ClassificationStatus::ResourceLimit);
  Same(saved,result);
  limits={}; limits.max_occurrences=1;
  input=f.Input();
  EXPECT_EQ(Classify(input,&result,limits).status,ClassificationStatus::ResourceLimit);
  Same(saved,result);
}
TEST(TiedClassificationFailure,LateFieldsIdentityAndRangesPreserveResultAndRetry) {
  Fixture f;
  ClassificationResult result,reference;
  ASSERT_TRUE(Classify(f.Input(),&result)); reference=result;
  f.nodes.back().kinematics.rotation=18;
  EXPECT_EQ(Classify(f.Input(),&result).status,ClassificationStatus::NativeDomain);
  Same(reference,result);
  f.nodes.back().kinematics.rotation=0;
  const auto id=f.nodes.back().source_id;
  f.nodes.back().source_id=f.nodes.front().source_id;
  EXPECT_EQ(Classify(f.Input(),&result).status,ClassificationStatus::DuplicateIdentity);
  Same(reference,result); f.nodes.back().source_id=id;
  f.roles[0].masters.back()=static_cast<std::uint32_t>(f.nodes.size());
  EXPECT_EQ(Classify(f.Input(),&result).status,ClassificationStatus::InvalidInput);
  Same(reference,result); f.roles[0].masters.back()=static_cast<std::uint32_t>(f.nodes.size()-1);
  f.tetra={{0,1,2}}; f.tags.assign(f.nodes.size(),0); f.tags.back()=INT_MIN;
  EXPECT_EQ(Classify(f.Input(),&result).status,ClassificationStatus::InvalidInput);
  Same(reference,result); f.tetra.clear(); f.tags.clear();
  ASSERT_TRUE(Classify(f.Input(),&result)); Same(reference,result);
}
TEST(TiedClassificationFailure,ExactBudgetOldResultBorrowAndRigidFailure) {
  Fixture f;
  ClassificationResult result;
  ASSERT_TRUE(Classify(f.Input(),&result));
  const auto reference=result;
  // The second operation accounts for both the old retained result and staging.
  ClassificationResult probe=result;
  ASSERT_TRUE(Classify(f.Input(),&probe));
  ClassificationLimits limits; limits.max_host_bytes=probe.startup_payload_bytes()-1;
  EXPECT_EQ(Classify(f.Input(),&result,limits).status,ClassificationStatus::ResourceLimit);
  Same(reference,result);
  ++limits.max_host_bytes;
  ASSERT_TRUE(Classify(f.Input(),&result,limits)); Same(reference,result);
  auto input=f.Input(); input.context.nodes=reference.nodes();
  // An old output may be borrowed by the next operation, including same object.
  ClassificationResult borrowed=reference;
  input.context.nodes=borrowed.nodes();
  ASSERT_TRUE(Classify(input,&borrowed));
  EXPECT_EQ(borrowed.irupt().data[0],1); // Existing interface condition is real.
  f.groups={{0},{static_cast<std::uint32_t>(f.nodes.size())}};
  EXPECT_EQ(RegisterRigidMembers(f.Rigid(),&result).status,ClassificationStatus::InvalidInput);
  Same(reference,result);
  f.groups.back()={1};
  ASSERT_TRUE(RegisterRigidMembers(f.Rigid(),&result));
  EXPECT_EQ(result.nodes().data[1].kinematics.conditions,8);
}
} // namespace classification_test
