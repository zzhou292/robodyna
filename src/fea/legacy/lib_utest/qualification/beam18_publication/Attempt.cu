// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace beam18_publication_test {
bool Rig::Begin(fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
  return mixed.Begin(token,view) && Good(beam.AssembleAccepted(mixed.owner,token,view));
}
bool Rig::Evaluate(const fe::NodalTrialToken& token,const fe::NodalPreparedView& view,
    fe::ShellPhysicalDiagnostics& d,bool with_beam) {
  return mixed.Evaluate(token,view,d) && (!with_beam || Good(beam.EvaluateCandidate(mixed.owner,token,view,&d.beam18)));
}
bool Rig::Prepare(fe::NodalTrialToken& token,fe::NodalPreparedView& view,fe::ShellPhysicalDiagnostics& output) {
  fe::NodalAssemblyView assembly; fe::ShellPhysicalDiagnostics candidates;
  if (!Begin(token,assembly) || !mixed.Advance(token,assembly,view) || !Evaluate(token,view,candidates)) return false;
  return Good(mixed.publication.PreparePhysical(mixed.owner,token,Candidates(candidates),&output));
}
bool Rig::Read(Snapshot& output) {
  Snapshot next;
  if (!mixed.Read(next.mixed)) return false;
  std::vector<b::Result> values(source.model.parents().size()); b::BatchDiagnostics diagnostics;
  if (!Good(beam.CopyAcceptedResults(mixed.owner.accepted(),{values.data(),values.size()},&diagnostics))) return false;
  EXPECT_TRUE(b::batch_detail::SameDiagnostics(diagnostics,next.mixed.diagnostics.beam18));
  next.beam=BeamValues(values);
  output=std::move(next);
  return true;
}
} // namespace beam18_publication_test
