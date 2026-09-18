// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace physical_publication_test {
bool Rig::Begin(fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
  return Good(owner.BeginTrial(&token,&view)) &&
      Good(qeph.AssembleMappedAccepted(owner,token,view)) &&
      Good(t3.AssembleMappedAccepted(owner,token,view)) &&
      Good(qbat.AssembleMappedAccepted(owner,token,view)) &&
      Good(welds.AssembleMappedAccepted(owner,token,view)) &&
      Good(beams.AssembleMappedAccepted(owner,token,view)) &&
      Good(solids.AssembleAccepted(owner,token,view));
}
bool Rig::Advance(const fe::NodalTrialToken& token,const fe::NodalAssemblyView& assembly,
    fe::NodalPreparedView& prepared) {
  fe::NodalCinAssemblyView cin;
  if (!Good(owner.BorrowCinAssembly(token,&cin))) return false;
  if (!Good(publication.ValidateAcceptedActivitySources(owner,Shells(),fixture.source.shells.inventory()))) return false;
  std::uint8_t q[2]{},b[1]{};
  fe::qeph::BatchDiagnostics qdiag;
  fe::qbat::BatchDiagnostics bdiag;
  if (!Good(qeph.CopyAcceptedParentActivity(owner.accepted(),q,2,&qdiag)) ||
      !Good(qbat.CopyAcceptedParentActivity(owner.accepted(),b,1,&bdiag))) return false;
  const std::uint8_t activity[]{std::uint8_t(q[0] ? 1 : 2),std::uint8_t(q[1] ? 1 : 2),std::uint8_t(b[0] ? 1 : 2)};
  // Exactly the retained Q/Q or Q/Q/B roster. All stiffness and internal loads were
  // assembled by the actual participants above; no fixture substitute exists.
  if (cudaMemcpyAsync(cin.witness_activity,activity,fixture.WitnessCount()*sizeof(activity[0]),cudaMemcpyHostToDevice,assembly.stream) != cudaSuccess)
    return false;
  const auto node = fixture.domain.Find(9305);
  double force = 0;
  if (cudaMemcpyAsync(&force,assembly.forces.force_x+node,sizeof(force),cudaMemcpyDeviceToHost,assembly.stream) != cudaSuccess ||
      cudaStreamSynchronize(assembly.stream) != cudaSuccess) return false;
  force += 10;
  if (cudaMemcpyAsync(assembly.forces.force_x+node,&force,sizeof(force),cudaMemcpyHostToDevice,assembly.stream) != cudaSuccess ||
      cudaStreamSynchronize(assembly.stream) != cudaSuccess) return false;
  if (external_force_source_node) {
    const auto external = fixture.domain.Find(external_force_source_node);
    if (external == SIZE_MAX) return false;
    double force_z = 0;
    if (cudaMemcpyAsync(&force_z, assembly.forces.force_z + external,
                        sizeof(force_z), cudaMemcpyDeviceToHost,
                        assembly.stream) != cudaSuccess ||
        cudaStreamSynchronize(assembly.stream) != cudaSuccess)
      return false;
    force_z += external_force_z_n;
    if (cudaMemcpyAsync(assembly.forces.force_z + external, &force_z,
                        sizeof(force_z), cudaMemcpyHostToDevice,
                        assembly.stream) != cudaSuccess ||
        cudaStreamSynchronize(assembly.stream) != cudaSuccess)
      return false;
  }
  return Good(owner.SealAssembly(token)) &&
      Good(fe::AdvanceStaggeredCin(owner,token,{assembly.owner_id,assembly.accepted.base_epoch,
          assembly.attempt,Qualification,H,.2,true})) &&
      Good(owner.BorrowPrepared(token,&prepared));
}
bool Rig::Evaluate(const fe::NodalTrialToken& token,const fe::NodalPreparedView& prepared,
    fe::ShellPhysicalDiagnostics& diagnostics,bool include_solids) {
  if (!Good(qeph.EvaluateCandidate(owner,token,prepared,&diagnostics.qeph)) ||
      !Good(t3.EvaluateCandidate(owner,token,prepared,&diagnostics.t3)) ||
      !Good(qbat.EvaluateCandidate(owner,token,prepared,&diagnostics.qbat)) ||
      !Good(welds.EvaluateCandidate(owner,token,prepared,&diagnostics.type25)) ||
      !Good(beams.EvaluateCandidate(owner,token,prepared,&diagnostics.type13))) return false;
  return !include_solids || Good(solids.EvaluateCandidate(owner,token,prepared,&diagnostics.solids));
}
bool Rig::Prepare(fe::NodalTrialToken& token,fe::NodalPreparedView& prepared,
    fe::ShellPhysicalDiagnostics& output) {
  fe::NodalAssemblyView assembly;
  fe::ShellPhysicalDiagnostics candidates;
  if (!Begin(token,assembly) || !Advance(token,assembly,prepared) || !Evaluate(token,prepared,candidates)) return false;
  return Good(publication.PreparePhysical(owner,token,{&candidates.qeph,&candidates.t3,&candidates.qbat,
      &candidates.type25,&candidates.type13,&candidates.solids},&output));
}
} // namespace physical_publication_test
