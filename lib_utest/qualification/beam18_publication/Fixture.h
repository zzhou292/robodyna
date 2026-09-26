// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Source.h"
#include "lib_utest/qualification/physical_publication/OwnerFixture.h"
#include "lib_src/elements/beam18/resident/Storage.h"
namespace beam18_publication_test {
using existing::Good;
struct Snapshot {
  existing::Snapshot mixed;
  std::vector<std::uint64_t> beam;
};
struct Rig {
  explicit Rig(fe::ShellBatchStartup startup={},bool independent_shell=false)
      : mixed(false,2.5,false,existing::ContactConstraintLayout::Legacy,false,.05,
              startup,independent_shell,independent_shell) {}
  Source source;
  b::Batch beam;
  // Destroy this actual publisher before beam and source above. Its six
  // participants retain their existing owner and lifetime order unchanged.
  existing::Rig mixed;
  bool Initialize(bool initialize_beam=true,bool attach=true);
  bool InitializeBeam(const b::Model* override_model=nullptr);
  bool Attach();
  fe::ShellPhysicalParticipants Participants();
  bool Begin(fe::NodalTrialToken&,fe::NodalAssemblyView&);
  bool Evaluate(const fe::NodalTrialToken&,const fe::NodalPreparedView&,fe::ShellPhysicalDiagnostics&,bool with_beam=true);
  bool Prepare(fe::NodalTrialToken&,fe::NodalPreparedView&,fe::ShellPhysicalDiagnostics&);
  bool Read(Snapshot&);
};
inline fe::ShellPhysicalCandidates Candidates(const fe::ShellPhysicalDiagnostics& d) {
  return {&d.qeph,&d.t3,&d.qbat,&d.type25,&d.type13,&d.solids,nullptr,&d.beam18};
}
inline void Exact(const Snapshot& a,const Snapshot& bvalue) {
  existing::Exact(a.mixed,bvalue.mixed);
  EXPECT_EQ(a.beam,bvalue.beam);
}
std::vector<std::uint64_t> BeamValues(const std::vector<b::Result>&);
} // namespace beam18_publication_test
