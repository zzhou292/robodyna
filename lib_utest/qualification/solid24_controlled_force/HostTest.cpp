// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace h24_full_test {
TEST(H24ControlledForce, CompleteNativeInitialAnd32StepHistoryInBothUnits){
 for(bool mm:{false,true}){SCOPED_TRACE(mm);auto x=Case(mm);const auto ref=Reference(x);NativeState native(x);auto accepted=Initial(x,ref,native);ASSERT_FALSE(HasFailure());
  for(unsigned step=0;step<32;++step){SCOPED_TRACE(step);q::Move(x,(step%8<4?1.:-1.)*(.1+.01*step));c::Scratch scratch;c::Result next;
   ASSERT_EQ(c::PrepareCandidate(ref,accepted.proposed_history,x.interval,scratch),s::ForceStatus::Success);ASSERT_EQ(c::Complete(scratch,scratch.activity.triggers_native_batch!=0,next),s::ForceStatus::Success);
   const auto n=native.Step(x);Compare(x,scratch,next,n);ASSERT_FALSE(HasFailure());accepted=next;native.Accept(n);x.interval.base_time_s+=x.interval.dt_s;++x.interval.sample_index;
  }
 }
}
TEST(H24ControlledForce, FoldedGeometryReachesNativeDistortionAndFinalSTI){
 for(bool mm:{false,true}){auto x=Case(mm);const auto ref=Reference(x);NativeState native(x);auto initial=Initial(x,ref,native);ASSERT_FALSE(HasFailure());
  const unsigned a=x.reference.source_slot(0),o=x.reference.source_slot(6);const auto first=x.reference.input().position_m[a],opposite=x.reference.input().position_m[o];
  x.interval.position_m[a]={first.x+.5*(opposite.x-first.x),first.y+.5*(opposite.y-first.y),first.z+1.005*(opposite.z-first.z)};
  x.interval.velocity_m_s[a]={0,0,.1*(opposite.z-first.z)};x.interval.velocity_m_s[x.reference.source_slot(1)]={0,0,-.05*(opposite.z-first.z)};
  c::Scratch scratch;c::Result result;ASSERT_EQ(c::PrepareCandidate(ref,initial.proposed_history,x.interval,scratch),s::ForceStatus::Success);ASSERT_EQ(c::Complete(scratch,scratch.activity.triggers_native_batch!=0,result),s::ForceStatus::Success);
  const auto n=native.Step(x);Compare(x,scratch,result,n);EXPECT_GT(result.center_contacts+result.corner_contacts,0);EXPECT_NE(result.distortion_energy_j,0);
 }
}
TEST(H24ControlledForce, RequiredBatchDecisionAndClockFailureRemainAtomic){auto x=Case(true);const auto ref=Reference(x);NativeState native(x);const auto initial=Initial(x,ref,native);ASSERT_FALSE(HasFailure());q::Move(x);
 c::Scratch scratch;c::Result result=initial;const auto before=Values(result);
 ASSERT_EQ(c::PrepareCandidate(ref,initial.proposed_history,x.interval,scratch),s::ForceStatus::Success);
 if(scratch.activity.triggers_native_batch){EXPECT_NE(c::Complete(scratch,false,result),s::ForceStatus::Success);EXPECT_EQ(Values(result),before);}
 ASSERT_EQ(c::Complete(scratch,scratch.activity.triggers_native_batch!=0,result),s::ForceStatus::Success);
 x.interval.sample_index+=1;EXPECT_EQ(c::PrepareCandidate(ref,initial.proposed_history,x.interval,scratch),s::ForceStatus::InvalidInput);EXPECT_FALSE(scratch.valid);
 const auto saved=Values(result);EXPECT_EQ(c::Complete(scratch,true,result),s::ForceStatus::InvalidInput);EXPECT_EQ(Values(result),saved);
 --x.interval.sample_index;ASSERT_EQ(c::PrepareCandidate(ref,initial.proposed_history,x.interval,scratch),s::ForceStatus::Success);
 auto other=Case(false);EXPECT_EQ(c::PrepareCandidate(Reference(other),initial.proposed_history,x.interval,scratch),s::ForceStatus::ReferenceMismatch);
}
TEST(H24ControlledForce, NonfiniteCandidateCannotPublishAndRepairs){auto x=Case(true);const auto ref=Reference(x);NativeState native(x);auto result=Initial(x,ref,native);ASSERT_FALSE(HasFailure());const auto before=Values(result);c::Scratch scratch;
 x.interval.velocity_m_s[2].x=std::numeric_limits<double>::infinity();EXPECT_EQ(c::PrepareCandidate(ref,result.proposed_history,x.interval,scratch),s::ForceStatus::InvalidInput);EXPECT_FALSE(scratch.valid);EXPECT_EQ(c::Complete(scratch,true,result),s::ForceStatus::InvalidInput);EXPECT_EQ(Values(result),before);
 x.interval.velocity_m_s[2].x=0;ASSERT_EQ(c::PrepareCandidate(ref,result.proposed_history,x.interval,scratch),s::ForceStatus::Success);EXPECT_EQ(c::Complete(scratch,scratch.activity.triggers_native_batch!=0,result),s::ForceStatus::Success);
}
} // namespace h24_full_test
