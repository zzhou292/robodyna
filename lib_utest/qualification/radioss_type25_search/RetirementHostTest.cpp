// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>
namespace type25_search_test {
TEST(Type25SearchRetirement, CompleteNativeXsaveKeepsSecondaryButSkipsRetiredMainRoles) {
  for(bool compact:{false,true})for(bool si:{false,true}){
    Fixture f(compact);f.EnableRetirement();if(si)f.ToSi();
    const auto before=NativeReference(f.source,f.Current());
    f.stiffness[0]=0;f.main_activity[f.main.back()]=0;
    const auto after=NativeReference(f.source,f.Current());
    ASSERT_EQ(after.size(),before.size());
    const double scale=si?f.source.units.length_m:1;
    // Original XSAVE has no STFN argument: NSV is still saved at its own slot.
    const auto slot=compact?0u:f.secondary[0];const auto node=f.secondary[0];
    for(unsigned k=0;k<3;++k)Same(after[3*slot+k],f.positions[3*node+k]/scale);
    if(compact)for(unsigned k=0;k<3;++k)EXPECT_EQ(after[3*(f.secondary.size()+f.main.size()-1)+k],0.);
    // The actual native current main mask is excluded from budget extrema.
    const auto e=NativeExtrema(f.source,f.Current(),f.Current());
    EXPECT_EQ(e.main_uses,f.main.size());
  }
}
TEST(Type25SearchRetirement, EmptyLiveSidesFollowNativeSentinelBudgetOnlyInExplicitProfile) {
  for(unsigned empty=1;empty<4;++empty){
    Fixture f;f.EnableRetirement();f.one_d.assign(2,UINT32_MAX);f.Bind();
    if(empty&1)for(auto& k:f.stiffness)k=0;
    if(empty&2)for(auto& a:f.main_activity)a=0;
    const auto e=NativeExtrema(f.source,f.Current(),f.Current());
    for(bool forced:{false,true})for(double dt:{0.,1e-5}){
      s::Budget out;out.distance=97;
      EXPECT_EQ(s::EvaluateBudget(e,4,dt,forced,out),s::Status::UnsupportedLifecycle);
      EXPECT_EQ(out.distance,97);
      ASSERT_EQ(s::EvaluateBudget(e,4,dt,forced,s::ActivityPolicy::MonotoneRetirement,out),s::Status::Ok);
      Same(out,NativeBudget(e,4,dt,forced));
    }
  }
}
TEST(Type25SearchRetirement, EmptySentinelsAndExplicitPolicyAreValidatedWithoutOutputMutation) {
  s::Extrema e;e.maximum_gap_change=0;s::Budget out;out.distance=71;
  ASSERT_EQ(s::EvaluateBudget(e,1,1,false,s::ActivityPolicy::MonotoneRetirement,out),s::Status::Ok);
  e.secondary_velocity.maximum.x=0;out.distance=71;
  EXPECT_EQ(s::EvaluateBudget(e,1,1,false,s::ActivityPolicy::MonotoneRetirement,out),s::Status::NonfiniteResult);
  EXPECT_EQ(out.distance,71);
  e={};e.maximum_gap_change=0;
  EXPECT_EQ(s::EvaluateBudget(e,1,1,false,static_cast<s::ActivityPolicy>(77),out),s::Status::InvalidInput);
  EXPECT_EQ(out.distance,71);
}
TEST(Type25SearchRetirement, PairedRoleMasksHaveExactSeparateResourceAdmission) {
  Fixture f;s::Forecast legacy,next;
  ASSERT_EQ(s::Maintenance::Preflight(f.source,{},legacy),s::Status::Ok);
  f.EnableRetirement();
  ASSERT_EQ(s::Maintenance::Preflight(f.source,{},next),s::Status::Ok);
  EXPECT_GT(next.device_bytes,legacy.device_bytes);
  s::Limits limits;limits.max_device_bytes=next.device_bytes;limits.max_host_bytes=next.startup_host_bytes;
  s::Forecast exact;
  EXPECT_EQ(s::Maintenance::Preflight(f.source,limits,exact),s::Status::Ok);
  EXPECT_EQ(exact.device_bytes,next.device_bytes);
  --limits.max_device_bytes;exact.device_bytes=59;
  EXPECT_EQ(s::Maintenance::Preflight(f.source,limits,exact),s::Status::ResourceLimit);
  EXPECT_EQ(exact.device_bytes,59u);
  f.source.activity_policy=static_cast<s::ActivityPolicy>(77);
  EXPECT_EQ(s::Maintenance::Preflight(f.source,{},exact),s::Status::InvalidInput);
}
}
