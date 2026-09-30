// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Packet.h"
namespace tl::fea::cin_parallel_test {
enum class Fault {
  AngleBeforeInverse, InverseBeforeAngle, PendingBeforeMotion, GeometryBeforeMotion,
  ScreenBeforeMotion, OrdinaryBeforeGroup, GroupZeroOrientationBeforeGroupOnePrimary
};
inline void Inject(Packet& p, Fault fault) {
  const auto tail = p.TailOffset();
  if (fault == Fault::InverseBeforeAngle) {
    p.trial[tail+127] = 0;
    p.accepted[6*Nodes+3*256] = 1e8;
  } else if (fault == Fault::GroupZeroOrientationBeforeGroupOnePrimary) {
    const auto first = p.members[0].node;
    p.accepted[9*Nodes+4*first] = 0;
    p.groups[1].mass = 0;
  } else {
    p.accepted[6*Nodes+3*127] = 1e8;
    p.trial[tail+256] = 0;
    if (fault == Fault::PendingBeforeMotion) p.activity.back() = 2;
    if (fault == Fault::GeometryBeforeMotion) {
      for (const auto node : p.rows.back().masters) {
        for (unsigned a=0; a<3; ++a) p.accepted[3*node+a] = 0;
      }
    }
    if (fault == Fault::ScreenBeforeMotion) {
      // Keep the later inverse valid so the structural screen reaches a bound.
      p.trial[tail+256] = p.accepted[tail+256];
      p.work[260] = 1e30;
      p.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
    }
    if (fault == Fault::OrdinaryBeforeGroup) p.groups[0].mass = 0;
  }
}
} // namespace tl::fea::cin_parallel_test
