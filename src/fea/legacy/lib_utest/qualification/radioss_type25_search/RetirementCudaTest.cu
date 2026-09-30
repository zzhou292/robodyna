// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace type25_search_test {
namespace {
constexpr auto Retirement=s::ReferenceCapturePolicy::MonotoneRoleRetirement;
void Replace(DeviceFixture& d) {
  s::ReferenceToken token;
  ASSERT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::Ok);
  ASSERT_EQ(d.owner.PublishReference(token),s::Status::Ok);
}
}
TEST_F(Type25SearchCuda, ExplicitRoleRetirementMatchesCompleteNativeReferenceAndExtrema) {
  for(bool compact:{false,true})for(bool si:{false,true})for(bool gaps:{false,true}){
    DeviceFixture d(compact,gaps,si,32,true);d.Initialize();d.Reference();
    d.fixture.stiffness[0]=0;d.fixture.main_activity[d.fixture.main.back()]=0;
    d.fixture.positions[3*d.fixture.secondary.back()]+=.125*(si?.001:1.);
    d.fixture.velocities[3*d.fixture.secondary.back()]=2*(si?.001:1.);
    ++d.fixture.stamp.epoch;++d.fixture.stamp.attempt;d.Upload();
    s::Report prior;prior.budget.distance=99;s::ReferenceToken old;
    EXPECT_EQ(d.owner.Evaluate(d.current,1e-5,false,prior),s::Status::UnsupportedLifecycle);
    EXPECT_EQ(prior.budget.distance,99);
    EXPECT_EQ(d.owner.StageReference(d.current,old),s::Status::UnsupportedLifecycle);
    Replace(d);d.Compare(d.fixture.Current(),1e-5,true);
    const auto saved=d.fixture.positions;const auto mask=d.fixture.main_activity;
    auto reference=d.fixture.Current();reference.positions.data=saved.data();reference.main_node_activity=mask.data();
    d.fixture.positions[3*d.fixture.secondary.back()]+=.0625*(si?.001:1.);d.Upload();
    d.Compare(reference,1e-5,false);
  }
}
TEST_F(Type25SearchCuda, RepeatedRetirementsRefreshBothInternalSlabsAndReachNativeEmptyRoles) {
  DeviceFixture d(true,false,false,32,true);d.Initialize();d.Reference();
  for(unsigned step=0;step<5;++step){
    if(step==0)d.fixture.stiffness[0]=0;
    if(step==1)d.fixture.main_activity[1]=0;
    if(step==2)d.fixture.stiffness[1]=0;
    if(step==3)d.fixture.main_activity[3]=0;
    if(step==4)d.fixture.main_activity[4]=0;
    ++d.fixture.stamp.epoch;++d.fixture.stamp.attempt;d.Upload();Replace(d);
    EXPECT_EQ(d.owner.reference_generation(),step+2);
    d.Compare(d.fixture.Current(),1e-5,true);
  }
  s::Report report;
  ASSERT_EQ(d.owner.Evaluate(d.current,1e-5,false,report),s::Status::Ok);
  EXPECT_EQ(report.extrema.secondary_uses,0u);
  EXPECT_EQ(report.extrema.main_uses,0u);
  EXPECT_EQ(report.budget.distance,d.fixture.source.margin);
}
TEST_F(Type25SearchCuda, EmptyCurrentRolesMatchNativeForEachSideAndBothLayouts) {
  for(bool compact:{false,true})for(unsigned empty=1;empty<4;++empty){
    DeviceFixture d(compact,false,false,32,true);
    d.fixture.one_d.assign(2,UINT32_MAX);d.fixture.Bind();d.Upload();d.Initialize();d.Reference();
    if(empty&1)for(auto& k:d.fixture.stiffness)k=0;
    if(empty&2)for(auto& a:d.fixture.main_activity)a=0;
    ++d.fixture.stamp.attempt;d.Upload();Replace(d);
    d.Compare(d.fixture.Current(),0,false);d.Compare(d.fixture.Current(),1e-5,true);
  }
}
TEST_F(Type25SearchCuda, RetiredNonfiniteGeometryIsUnusedButRemainingLiveRolesStillValidate) {
  DeviceFixture d(true,false,false,32,true);d.Initialize();d.Reference();
  d.fixture.stiffness[0]=0;d.fixture.main_activity[1]=0;
  d.fixture.positions[0]=NAN;d.fixture.velocities[0]=NAN;
  d.fixture.positions[3]=NAN;d.fixture.velocities[3]=NAN;
  ++d.fixture.stamp.attempt;d.Upload();Replace(d);d.Compare(d.fixture.Current());
  s::ReferenceToken token;d.fixture.positions[3*3]=NAN;d.Upload();
  EXPECT_EQ(d.owner.StageReference(d.current,Retirement,token),s::Status::NonfiniteResult);
  EXPECT_EQ(d.owner.reference_generation(),2u);
}
}
