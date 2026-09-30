// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../type45_joint/Fixture.h"
#include "lib_src/elements/type45/resident/ResultChecks.h"
#include <limits>

namespace type45_test {
namespace resident=tl::fea::type45::resident_detail;
Joint Row(const Fixture& f) {
  Joint row;
  row.property=f.property;row.geometry=f.geometry;
  for(unsigned e=0;e<2;++e) row.damping[e]=f.damping[e];
  return row;
}
TEST(Type45ResidentState,FirstTrialOwnsAutomaticReferenceAndRejectedAttemptCannotReplaceIt) {
  for(auto kind:{Kind::Spherical,Kind::Revolute,Kind::Cylindrical}) {
    Fixture fixture(kind);const auto row=Row(fixture);
    resident::State accepted;
    ASSERT_EQ(resident::InitializeState(row,accepted),Status::Success);
    ASSERT_TRUE(resident::ValidResult(row,accepted,0,0,fixture.context.target_dt_s));
    auto interval=fixture.Step(accepted.history);
    interval.position_m[1].y+=1e-5;
    resident::State first;
    ASSERT_EQ(resident::UpdateState(row,accepted,&fixture.context,interval,first),Status::Success);
    EXPECT_FALSE(accepted.history.ready());
    EXPECT_TRUE(first.history.ready());
    auto bad_context=fixture.context;bad_context.target_dt_s*=2;
    EXPECT_EQ(resident::UpdateState(row,accepted,&bad_context,interval,first),Status::InvalidContext);
    EXPECT_TRUE(resident::ValidResult(row,first,interval.dt_s,1,fixture.context.target_dt_s));
    // Only this assignment represents acceptance in the value fixture.
    accepted=first;
    interval=fixture.Step(accepted.history);interval.position_m[1].y+=2e-5;
    resident::State next;
    EXPECT_EQ(resident::UpdateState(row,accepted,&fixture.context,interval,next),Status::InvalidContext);
    ASSERT_EQ(resident::UpdateState(row,accepted,nullptr,interval,next),Status::Success);
    EXPECT_TRUE(next.history.reference().Matches(accepted.history.reference()));
    EXPECT_TRUE(resident::ValidResult(row,next,2*interval.dt_s,2,fixture.context.target_dt_s));
  }
}
TEST(Type45ResidentState,AllNamedRecurrenceFieldsMatchLeafWithLateInputRollbackAndRetry) {
  Fixture f;const auto row=Row(f);resident::State state;
  ASSERT_EQ(resident::InitializeState(row,state),Status::Success);
  const auto reference=f.Prepare();History leaf;
  ASSERT_EQ(History::Initialize(reference,leaf),Status::Success);
  for(unsigned step=0;step<4;++step) {
    auto interval=f.Step(leaf);interval.position_m[1].y+=1e-6*(step+1);
    interval.angular_velocity_rad_s[1]={.001,-.002,.003};
    resident::State next=state;
    auto bad=interval;bad.angular_velocity_rad_s[1].z=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(resident::UpdateState(row,state,step?nullptr:&f.context,bad,next),Status::InvalidGeometry);
    EXPECT_EQ(next.history.stamp().sample_index,state.history.stamp().sample_index);
    ASSERT_EQ(resident::UpdateState(row,state,step?nullptr:&f.context,interval,next),Status::Success);
    Evaluation expected;
    ASSERT_EQ(Evaluate(reference,leaf,interval,expected),Status::Success);
    Evaluation observed;observed.history=next.history;observed.diagnostics=next.cache.diagnostics;
    for(unsigned e=0;e<2;++e) observed.endpoint[e]=next.cache.endpoint[e];
    Same(observed,expected);
    state=next;leaf=expected.history;
  }
  auto changed=row;changed.damping[1].mass_kg+=1;
  resident::State output=state;
  EXPECT_EQ(resident::UpdateState(changed,state,nullptr,f.Step(leaf),output),Status::ReferenceMismatch);
  EXPECT_FALSE(resident::ValidResult(changed,state,leaf.stamp().time_s,4,f.context.target_dt_s));
  output.cache.endpoint[1].couple_nm.z=std::numeric_limits<double>::infinity();
  Result result;result.source_joint_id=999;
  EXPECT_FALSE(resident::Export(row,output,result));
  EXPECT_EQ(result.source_joint_id,999u);
}
} // namespace type45_test
