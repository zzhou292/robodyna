// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_cooperative_group_screen/Fixture.h"
#include "lib_src/solvers/cin_advance/group_motion/Primary.h"
#include "lib_src/solvers/cin_advance/group_motion/Wrench.h"
#include "lib_src/solvers/cin_advance/group_motion/Members.h"
#include "lib_src/solvers/cin_advance/group_motion/Orientation.h"
#include <limits>
namespace tl::fea::group_motion_test {
namespace packet=cin_parallel_test;
namespace motion=cin_advance::group_motion;
namespace groups=cin_advance::groups;
using cooperative_test::Members;
using cooperative_test::Same;
// Reuse the qualified source packet; this host schedule deliberately prepares
// each tile in reverse lane order before the unchanged source-order scan.
template<bool Capture>
inline groups::Report Tiled(const cin_advance::Input& in,unsigned group) {
  motion::Tile tile{};tile.state=motion::Begin<Capture>(in,group);
  if (motion::Failed(tile.state)) return tile.state.report;
  for (unsigned first=0;first<tile.state.count&&!motion::Failed(tile.state);) {
    const unsigned count=std::min<unsigned>(motion::Threads,tile.state.count-first);
    for (unsigned lane=count;lane-- >0;)tile.wrench[lane]=motion::PrepareWrench(in,tile.state,first+lane);
    for (unsigned lane=0;lane<count&&!motion::Failed(tile.state);++lane)motion::FoldWrench(tile.wrench[lane],tile.state);
    first+=count;
  }
  if (motion::Failed(tile.state)) return tile.state.report;
  motion::PreparePrimary(in,tile.state);
  if (motion::Failed(tile.state)) return tile.state.report;
  for (unsigned first=0;first<tile.state.count&&!motion::Failed(tile.state);) {
    const unsigned count=std::min<unsigned>(motion::Threads,tile.state.count-first);
    rigid::MemberStepTrial next[motion::Threads];
    for (unsigned lane=count;lane-- >0;)tile.status[lane]=motion::PrepareMember(in,tile.state,first+lane,next[lane]);
    motion::SelectPrefix(in,first,count,tile);
    for (unsigned lane=0;lane<tile.state.prefix;++lane)motion::PublishMember<Capture>(in,tile.state,first+lane,next[lane]);
    first+=count;
  }
  if (motion::Failed(tile.state)) return tile.state.report;
  motion::PublishGroup<Capture>(in,group,tile.state);
  for (unsigned first=0;first<tile.state.count&&!motion::Failed(tile.state);) {
    const unsigned count=std::min<unsigned>(motion::Threads,tile.state.count-first);
    tl::math::Quaternion next[motion::Threads];
    for (unsigned lane=count;lane-- >0;)tile.status[lane]=motion::PrepareOrientation(in,tile.state,first+lane,next[lane]);
    motion::SelectPrefix(in,first,count,tile);
    for (unsigned lane=0;lane<tile.state.prefix;++lane)motion::PublishOrientation(in,tile.state,first+lane,next[lane]);
    first+=count;
  }
  return tile.state.report;
}
inline groups::Report Tiled(const cin_advance::Input& in,unsigned group) {
  return in.capture.node?Tiled<true>(in,group):Tiled<false>(in,group);
}
inline void SamePrivate(const packet::Packet& before,const packet::Packet& a,const packet::Packet& b) {
  packet::SameDoubles(a.accepted,before.accepted);packet::SameDoubles(b.accepted,before.accepted);
  packet::SameDoubles(a.trial,b.trial);packet::SameDoubles(a.capture,b.capture);
  packet::SameDoubles(a.loads,b.loads);packet::SameDoubles(a.work,b.work);
}
inline unsigned Node(const packet::Packet& p,unsigned local) {return p.members[local].node;}
inline void Fault(packet::Packet& p,unsigned fault,unsigned local=64) {
  const auto first=Node(p,0),late=Node(p,local);
  if (fault==0) {p.groups[0].mass=0;p.members[local].node=packet::Nodes;}
  if (fault==1) {p.members[0].mass=-1;p.loads[late]=std::numeric_limits<double>::quiet_NaN();}
  if (fault==2) {
    const auto state=rigid::ReadGroupState(p.accepted.data()+19*packet::Nodes);
    for (unsigned k=0;k<2;++k) {
      const auto n=Node(p,k);rigid::candidate_detail::WriteNode(p.accepted.data(),n,state.center);
      for (unsigned a=0;a<6;++a)p.loads[a*packet::Nodes+n]=0;
      p.loads[n]=std::numeric_limits<double>::max();
    }
    p.members[local].node=packet::Nodes;
  }
  if (fault==3) {p.members[local].mass=-1;p.accepted[9*packet::Nodes+4*first]=0;}
  if (fault==4)p.accepted[9*packet::Nodes+4*late]=0;
  if (fault==5)p.accepted[3*packet::Nodes+3*late]=std::numeric_limits<double>::quiet_NaN();
  if (fault==6)p.groups[0].count=1;
  if (fault==7)p.groups[0].offset=UINT32_MAX;
  p.Begin(1);
}
} // namespace tl::fea::group_motion_test
