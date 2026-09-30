// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../cin_parallel_groups/FrozenMotion.h"
namespace tl::fea::group_motion_test {
TEST(CinCooperativeMotionHost, ReversePreparedTilesMatchOriginalGroupAcrossBoundaries) {
  for (bool capture:{false,true})for (unsigned count:{2,3,63,64,65,127,128,129,260}) {
    SCOPED_TRACE(capture);
    SCOPED_TRACE(count);
    auto before=Members({count});before.capture_enabled=capture;
    for (unsigned step=0;step<3;++step) {
      if(step==1)std::reverse(before.members.begin(),before.members.end());
      before.Begin(step+1);auto old=before,now=before;
      const auto expected=groups::AdvanceGroup(old.Input(),0),actual=Tiled(now.Input(),0);
      ASSERT_EQ(expected.status,NodalStatus::Ok);Same(expected,actual);SamePrivate(before,old,now);
      before=now;before.Accept();
    }
  }
}
TEST(CinCooperativeMotionHost, OrderedFailureAndEveryPrivatePrefixMatchOriginalBody) {
  for (bool capture:{false,true})for (unsigned local:{0,63,64,65,128})for (unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(capture);
    SCOPED_TRACE(local);
    SCOPED_TRACE(fault);
    // The cumulative-overflow case needs two valid predecessors before the bad node.
    if (fault==2&&local<2)continue;
    auto before=Members({129});before.capture_enabled=capture;Fault(before,fault,local);
    auto old=before,now=before;
    const auto expected=groups::AdvanceGroup(old.Input(),0),actual=Tiled(now.Input(),0);
    ASSERT_NE(expected.status,NodalStatus::Ok);Same(expected,actual);SamePrivate(before,old,now);
    if (fault==0)EXPECT_EQ(actual.last_node,packet::Nodes);
    if (fault==2)EXPECT_EQ(actual.last_node,Node(before,1));
    if (fault==3||fault==4||fault==5)EXPECT_EQ(actual.last_node,Node(before,local));
  }
  auto before=Members({129}),prepared=before;
  auto input=prepared.Input();
  ASSERT_EQ(rigid::PrepareGroupCandidate<true>(input.groups,0,input.accepted,input.trial,
      input.loads,input.model.node_count,input.durations,input.capture).status,rigid::StepStatus::Success);
  tl::math::Quaternion candidate;
  ASSERT_EQ(nodal_detail::PrepareNodeOrientationValue(input.accepted,input.trial,Node(before,0),
      packet::Nodes,input.durations.drift_dt,0,false,candidate),NodalStatus::StepTooLarge);
  auto old=before,now=before;auto a=old.Input(),b=now.Input();
  a.maximum_angle=b.maximum_angle=0;
  const auto expected=groups::AdvanceGroup(a,0),actual=Tiled(b,0);
  ASSERT_EQ(expected.status,NodalStatus::StepTooLarge);
  EXPECT_EQ(expected.last_node,Node(before,0));Same(expected,actual);SamePrivate(before,old,now);
  // All motion/capture/group fields were written before the first quaternion rejected.
  packet::SameDoubles(now.trial,prepared.trial);packet::SameDoubles(now.capture,prepared.capture);
}
TEST(CinCooperativeMotionHost, CaptureAdmissionAndDisabledStrayPointersPreserveDefaults) {
  for (unsigned mode=0;mode<5;++mode) {
    auto before=Members({65}),old=before,now=before;auto a=old.Input(),b=now.Input();
    if (mode==0) {a.capture.node=b.capture.node=nullptr;} // Other sinks ignored.
    if (mode==1) {a.capture.node_rotation=b.capture.node_rotation=nullptr;}
    if (mode==2) {a.capture.group=b.capture.group=nullptr;}
    if (mode==3) {a.capture.group_rotation=b.capture.group_rotation=nullptr;}
    if (mode==4) {a.groups.groups=b.groups.groups=nullptr;}
    const auto expected=groups::AdvanceGroup(a,0),actual=Tiled(b,0);
    Same(expected,actual);SamePrivate(before,old,now);
    EXPECT_EQ(actual.first_node,UINT32_MAX);EXPECT_EQ(actual.minimum_dt,0);
    EXPECT_FALSE(actual.visited);EXPECT_FALSE(actual.bounded);
    if (mode) {EXPECT_EQ(actual.status,NodalStatus::InvalidOutput);EXPECT_EQ(actual.last_node,UINT32_MAX);}
    else EXPECT_EQ(actual.status,NodalStatus::Ok);
  }
}
TEST(CinCooperativeMotionHost, TwoMemberSourceUnitsAndZeroDependentCoefficientsRemainNative) {
  for (double units:{.001,1.,0.})for (bool capture:{false,true}) {
    auto before=Members({2});before.capture_enabled=capture;
    before.members[1].mass=before.members[1].inertia=0;before.Begin(1);
    auto old=before,now=before;auto a=old.Input(),b=now.Input();
    a.groups.source_length_to_m=b.groups.source_length_to_m=units;
    const auto expected=groups::AdvanceGroup(a,0),actual=Tiled(b,0);
    Same(expected,actual);SamePrivate(before,old,now);
    EXPECT_EQ(actual.status,units>0?NodalStatus::Ok:NodalStatus::InvalidOutput);
  }
}
} // namespace tl::fea::group_motion_test
