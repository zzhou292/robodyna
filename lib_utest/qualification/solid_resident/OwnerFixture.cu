// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace solid_resident_test {
bool Rig::Initialize(bool claim) {
  auto& f=fixture.mechanics;
  f.DependentInverses(true);
  const auto cin=f.Cin();
  if (!Good(owner.Initialize(f.Config(),f.Kinematics(),f.im.data(),f.Dofs(),f.binding,&cin))) return false;
  config=fixture.Configuration();
  config.owner=owner.accepted();
  if (!Good(batch.InitializeJoined(config,fixture.model))) return false;
  if (!native.Initialize(fixture.model,{})) return false;
  return !claim || Attach();
}
bool Rig::Attach() {
  const auto& f=fixture.mechanics;
  if (!Good(Peer::PreflightAttach(batch,owner,f.ledger,f.binding,fixture.Witnesses(),fixture.model,config))) return false;
  Peer::Attach(batch);
  return true;
}
bool Rig::Begin(fe::NodalTrialToken& token,fe::NodalAssemblyView& view) {
  return Good(owner.BeginTrial(&token,&view)) && Good(batch.AssembleAccepted(owner,token,view));
}
bool Rig::Prepare(const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view,
    fe::NodalPreparedView& prepared) {
  fe::NodalCinAssemblyView cin;
  if (!Good(owner.BorrowCinAssembly(token,&cin))) return false;
  // Disjoint fixture CIN patch: explicit other-producer prescribed stiffness
  // and witness activity. These do not assert other resident participants.
  std::vector<double> stiffness(cin.node_count);
  if (cudaMemcpyAsync(stiffness.data(),cin.translational_stiffness,stiffness.size()*sizeof(double),
      cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess || cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  const auto rows=fixture.mechanics.cin_model.rows();
  for (std::size_t k=0;k<rows.count;++k) {
    const auto& row=rows.data[k];
    for (auto node:row.master_domain_nodes) stiffness[node]+=1;
    stiffness[row.secondary_domain_node]+=1;
  }
  if (cudaMemcpyAsync(cin.translational_stiffness,stiffness.data(),stiffness.size()*sizeof(double),
      cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess ||
      cudaMemsetAsync(cin.witness_activity,1,cin.witness_count,cin.stream)!=cudaSuccess) return false;
  // An actual owner drift strains the shared solid topology; no prescribed
  // candidate/history override participates in a successful recurrence.
  const auto node=fixture.mechanics.domain.Find(9305);
  double load=0;
  if (cudaMemcpyAsync(&load,view.forces.force_x+node,sizeof(load),cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess ||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  load+=10;
  if (cudaMemcpyAsync(view.forces.force_x+node,&load,sizeof(load),cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess ||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  if (!Good(owner.SealAssembly(token))) return false;
  if (!Good(fe::AdvanceStaggeredCin(owner,token,{view.owner_id,view.accepted.base_epoch,view.attempt,
      cin.qualification_id,config.owner.fixed_dt,.2,true}))) return false;
  return Good(owner.BorrowPrepared(token,&prepared));
}
bool Rig::Read(Results& results,s::BatchDiagnostics& diagnostics) {
  return Good(batch.CopyAcceptedResults(owner.accepted(),results.Buffers(),&diagnostics));
}
bool Rig::ComparePrepared(const fe::NodalPreparedView& view,const Results& results) {
  std::vector<double> x(3*config.owner.node_count),v(x.size());
  if (cudaMemcpyAsync(x.data(),view.kinematics.position_xyz,x.size()*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess ||
      cudaMemcpyAsync(v.data(),view.kinematics.velocity_xyz,v.size()*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess ||
      cudaStreamSynchronize(view.stream)!=cudaSuccess) return false;
  if (!native.Evaluate(x,v,view.base_time,config.owner.fixed_dt,view.kinematics.base_epoch)) return false;
  native.Compare(results,view.proposed_time,view.kinematics.base_epoch+1);
  return !::testing::Test::HasFailure();
}
} // namespace solid_resident_test
