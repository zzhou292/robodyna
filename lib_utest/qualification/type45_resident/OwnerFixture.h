// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../physical_publication/OwnerFixture.h"
#include "../type45_joint/NativeOracle.h"
#include "../type45_joint/Observation.h"
#include "lib_src/elements/type45/resident/Storage.h"

namespace tl::fea::type45 {
class BatchQualificationPeer {
 public:
  static Evaluation Staged(const Batch& batch,std::size_t j) {
    const auto& s=*batch.impl_;
    const auto* values=util::ArenaPointer<resident_detail::State>(s.staging.data(),s.layout.staging);
    Evaluation out;out.history=values[j].history;out.diagnostics=values[j].cache.diagnostics;
    for(unsigned e=0;e<2;++e) out.endpoint[e]=values[j].cache.endpoint[e];
    return out;
  }
};
}
namespace type45_resident_test {
namespace fe=tl::fea;
namespace joint=fe::type45;
namespace common=physical_publication_test;
using common::Good;
struct Rig {
  explicit Rig(fe::ShellBatchStartup startup={},bool independent_shell=false)
      : physical(false,2.5,false,common::ContactConstraintLayout::Legacy,false,.05,
                 startup,independent_shell,independent_shell) {}
  joint::Model model;
  joint::Batch joints;
  common::Rig physical; // Its publication is destroyed before the joint batch.
  joint::BatchConfig Config() const;
  fe::ShellPhysicalParticipants Participants();
  bool Initialize(joint::WorkingUnits units=joint::WorkingUnits::SI,bool attach=true,bool initialize_batch=true);
  bool Attach();
  bool Stage(fe::NodalTrialToken&,fe::NodalPreparedView&,fe::ShellPhysicalDiagnostics&,bool evaluate_joints=true);
  bool Prepare(fe::NodalTrialToken&,fe::NodalPreparedView&,fe::ShellPhysicalDiagnostics&);
  bool CopyAccepted(std::array<joint::Result,3>&,joint::BatchDiagnostics&);
};
inline fe::ShellPhysicalCandidates Candidates(const fe::ShellPhysicalDiagnostics& d) {
  return {&d.qeph,&d.t3,&d.qbat,&d.type25,&d.type13,&d.solids,&d.type45};
}
bool JointScatter(Rig&,const fe::NodalTrialToken&,const fe::NodalAssemblyView&);
joint::Interval Interval(const joint::Joint&,const fe::NodalPreparedView&,double dt);
std::vector<type45_test::NativeOracle> Native(Rig&,const fe::NodalTrialToken&,const fe::NodalPreparedView&);
void ComparePrepared(Rig&,const fe::NodalPreparedView&,const joint::BatchDiagnostics&,
    std::vector<type45_test::NativeOracle>&);
} // namespace type45_resident_test
