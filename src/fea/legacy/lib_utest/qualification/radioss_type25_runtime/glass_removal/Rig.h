// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Contact.h"
#include "../FullLedgerRig.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include "../../resident_shell_tab1/Values.h"
namespace glass_removal_test {
using type25_source_test::Check;
struct Attempt {
 fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
 fe::ShellPhysicalDiagnostics materials,common;
 std::array<fe::ShellPhysicalScratchParticipationReceipt,2> contacts;
};
struct State {
 std::array<double,33> x{},v{},omega{},reaction{},couple{};
 std::array<double,44> q{};std::array<double,11> mass{},inertia{};
 std::array<fe::qeph::ForceTrial,2> force;
 std::array<fe::ShellBatchLayeredSection,2> section;
 std::array<fe::ShellBatchFailureState,2> failure;
 fe::t3::ForceTrial triangle_force;fe::ShellBatchLayeredSection triangle_section;
 fe::ShellBatchFailureState triangle_failure;
 std::array<std::vector<n::NativeGeometryHistory>,2> contacts;
 std::array<std::vector<int>,2> flags;
 std::array<fe::NativeContactPublicationSnapshot,2> publication;
 fe::NodalStamp stamp;
};
struct Rig {
 static constexpr std::uint64_t Configuration=910300;
 static constexpr double Dt=1.5e-7;
 nodal_empty_test::Source source=Source();
 nodal_empty_test::Fixture base;
 fe::ShellBatchFailureBinding failure;
 fe::ShellPhysicalBinding physical;
 fe::FENodalState owner;fe::qeph::QephBatch qeph;fe::t3::T3Batch triangle;
 fe::ShellBatchPublication publication;
 ContactSource self_source,wall_source;
 n::Transaction self,wall;
 bool self_response_nonzero=false,wall_response_nonzero=false;
 fe::ShellPhysicalParticipants Participants(){return {&qeph,&triangle};}
 fe::ShellPhysicalPublicationIdentity Identity() const{return {Configuration,base.Qualification,base.startup};}
 void Initialize();
 void Begin(Attempt&,bool bending=false);
 void Assemble(Attempt&);
 void Prepare(Attempt&);
 void Seal(Attempt&);
 fe::ShellPublicationReport Commit(Attempt&,bool approved=true);
 void Discard();
 State Read();
 std::array<fe::ShellBatchFailureState,2> PreparedFailure(const Attempt&);
 std::vector<double> Force(const Attempt&);
 void Step(bool bending=false);
};
void Same(const State&,const State&);
unsigned FailedPoints(const fe::ShellBatchFailureState&);
}
