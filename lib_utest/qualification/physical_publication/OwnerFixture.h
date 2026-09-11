// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_src/elements/publication/PhysicalState.h"

namespace physical_publication_test {
template<class Report> bool Good(const Report& report) {
  using Status = decltype(report.status);
  EXPECT_EQ(report.status,Status::Success) << report.message;
  return report.status == Status::Success;
}
inline bool Good(const fe::NodalReport& report) {
  EXPECT_EQ(report.status,fe::NodalStatus::Ok) << report.message;
  return report.status == fe::NodalStatus::Ok;
}
struct Snapshot {
  fe::NodalStamp stamp;
  fe::ShellPhysicalDiagnostics diagnostics;
  std::vector<std::uint64_t> values;
};
struct Rig {
  explicit Rig(bool surface_rigid=false,double t3_failure=2.5) : fixture(surface_rigid,t3_failure) {}
  Fixture fixture;
  fe::FENodalState owner;
  fe::qeph::QephBatch qeph;
  fe::t3::T3Batch t3;
  fe::qbat::Batch qbat;
  fe::type25::Batch welds;
  fe::type13::Batch beams;
  fe::solids::Batch solids;
  fe::ShellBatchPublication publication; // Destroy before all borrowed objects.
  fe::ShellPhysicalParticipants Participants() { return {&qeph,&t3,&qbat,&welds,&beams,&solids}; }
  fe::ShellFormulationParticipants Shells() { return {&qeph,&t3,&qbat,&welds}; }
  bool Initialize(bool initialize_solids = true,bool attach = true);
  bool InitializeSolids();
  bool Attach();
  bool Begin(fe::NodalTrialToken&,fe::NodalAssemblyView&);
  bool Advance(const fe::NodalTrialToken&,const fe::NodalAssemblyView&,fe::NodalPreparedView&);
  bool Evaluate(const fe::NodalTrialToken&,const fe::NodalPreparedView&,fe::ShellPhysicalDiagnostics&,bool include_solids = true);
  bool Prepare(fe::NodalTrialToken&,fe::NodalPreparedView&,fe::ShellPhysicalDiagnostics&);
  bool Read(Snapshot&);
};
inline void Exact(const Snapshot& before,const Snapshot& after) {
  EXPECT_TRUE(fe::trial_identity::SameStamp(before.stamp,after.stamp));
  EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(before.diagnostics,after.diagnostics));
  EXPECT_EQ(before.values,after.values);
}
} // namespace physical_publication_test
