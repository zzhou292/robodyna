// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
#include "CopyFault.h"
namespace type25_search_test {
namespace {constexpr auto Retirement=s::ReferenceCapturePolicy::MonotoneRoleRetirement;}
TEST_F(Type25SearchCuda, RetirementNeedsExplicitSourceAndRejectsReactivationWithoutChangingReference) {
  DeviceFixture old;old.Initialize();old.Reference();s::ReferenceToken token;
  EXPECT_EQ(old.owner.StageReference(old.current,Retirement,token),s::Status::UnsupportedProfile);
  DeviceFixture d(true,true,false,32,true);d.Initialize();d.Reference();
  d.fixture.stiffness[0]=0;d.fixture.main_activity[1]=0;d.Upload();
  ASSERT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::Ok);
  d.owner.DiscardReference();
  EXPECT_EQ(d.owner.PublishReference(token),s::Status::StaleReference);
  EXPECT_EQ(d.owner.reference_generation(),1u);
  d.fixture.stiffness[0]=2;d.fixture.main_activity[1]=1;d.Upload();d.Compare(d.fixture.Current());
  d.fixture.stiffness[0]=0;d.fixture.main_activity[1]=0;d.Upload();
  ASSERT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::Ok);
  ASSERT_EQ(d.owner.PublishReference(token),s::Status::Ok);
  const auto committed=token.generation();
  for(unsigned field=0;field<2;++field){
    if(field==0)d.fixture.stiffness[0]=2;else d.fixture.main_activity[1]=1;
    d.Upload();
    EXPECT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::UnsupportedLifecycle);
    EXPECT_EQ(token.generation(),committed);
    EXPECT_EQ(d.owner.reference_generation(),committed);
    d.fixture.stiffness[0]=0;d.fixture.main_activity[1]=0;
  }
  d.Upload();d.Compare(d.fixture.Current());
}
TEST_F(Type25SearchCuda, RoleMaskShapeValuesStampsAndUnknownCaptureModesFailClosed) {
  DeviceFixture d(true,true,false,32,true);d.Initialize();d.Reference();s::ReferenceToken token;
  auto wrong=d.current;wrong.main_node_activity=nullptr;
  EXPECT_EQ(d.owner.StageReference(wrong,Retirement,token),s::Status::InvalidInput);
  wrong=d.current;--wrong.main_node_activity_count;
  EXPECT_EQ(d.owner.StageReference(wrong,Retirement,token),s::Status::InvalidInput);
  wrong=d.current;wrong.main_node_activity=reinterpret_cast<const std::uint8_t*>(UINTPTR_MAX-1);
  EXPECT_EQ(d.owner.StageReference(wrong,Retirement,token),s::Status::InvalidInput);
  wrong=d.current;++wrong.stamp.source.activity;
  EXPECT_EQ(d.owner.StageReference(wrong,Retirement,token),s::Status::UnsupportedLifecycle);
  wrong=d.current;wrong.stamp.attempt=0;
  EXPECT_EQ(d.owner.StageReference(wrong,Retirement,token),s::Status::StaleReference);
  EXPECT_EQ(d.owner.StageReference(d.current,static_cast<s::ReferenceCapturePolicy>(77),token),s::Status::InvalidInput);
  d.fixture.main_activity[1]=2;d.Upload();
  EXPECT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::UnsupportedLifecycle);
  EXPECT_TRUE(d.owner.last_failure().row_available);
  EXPECT_EQ(d.owner.last_failure().input_row,d.fixture.secondary.size());
  d.fixture.main_activity[1]=1;d.fixture.stiffness[0]=-1;d.Upload();
  EXPECT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::UnsupportedLifecycle);
  d.fixture.stiffness[0]=NAN;d.Upload();
  EXPECT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::NonfiniteResult);
  EXPECT_EQ(d.owner.reference_generation(),1u);
  d.fixture.stiffness[0]=2;d.Upload();d.Compare(d.fixture.Current());
}
TEST_F(Type25SearchCuda, RetirementOutputAliasAndCudaFailurePreservePublication) {
  DeviceFixture d(true,true,false,32,true);d.Initialize();d.Reference();s::ReferenceToken token;
  auto alias=d.current;alias.main_node_activity=reinterpret_cast<const std::uint8_t*>(&token);
  EXPECT_EQ(d.owner.StageReference(alias,Retirement,token),s::Status::InvalidInput);
  d.fixture.stiffness[0]=0;d.fixture.main_activity[1]=0;d.Upload();
  copy_fault::Arm();
  EXPECT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::DeviceFailure);
  EXPECT_EQ(d.owner.reference_generation(),1u);
  EXPECT_EQ(d.owner.PublishReference(token),s::Status::Unusable);
}
}
