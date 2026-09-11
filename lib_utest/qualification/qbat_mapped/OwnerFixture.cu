// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qbat_mapped_test {
bool Rig::Prepare(fe::NodalTrialToken& token,fe::NodalAssemblyView& assembly,fe::NodalPreparedView& prepared) {
  if (!Begin(token,assembly)) return false;
  fe::NodalCinAssemblyView cin;
  auto report=owner.BorrowCinAssembly(token,&cin);
  if (report.status!=fe::NodalStatus::Ok) return false;
  // Explicit prescribed-owner test inputs for the disjoint CIN patch. This is
  // not a claim that its other source producers have resident participants.
  std::vector<double> stiffness(cin.node_count);
  for (std::size_t row=0;row<fixture.mechanics.cin_model.rows().count;++row) {
    const auto& r=fixture.mechanics.cin_model.rows().data[row];
    for (auto node:r.master_domain_nodes) stiffness[node]=1;
    stiffness[r.secondary_domain_node]=1;
  }
  std::vector<double> actual(cin.node_count);
  if (cudaMemcpyAsync(actual.data(),cin.translational_stiffness,actual.size()*sizeof(double),
      cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess || cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  for (std::size_t node=0;node<actual.size();++node) actual[node]+=stiffness[node];
  if (cudaMemcpyAsync(cin.translational_stiffness,actual.data(),actual.size()*sizeof(double),
      cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess) return false;
  std::vector<std::uint8_t> activity(cin.witness_count,1);
  if (cudaMemcpyAsync(cin.witness_activity,activity.data(),activity.size(),cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess) return false;
  const double load=1000;
  const auto node=fixture.mechanics.domain.Find(12);
  if (cudaMemcpyAsync(assembly.forces.force_x+node,&load,sizeof(load),cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess ||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  report=owner.SealAssembly(token);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if (report.status!=fe::NodalStatus::Ok) return false;
  report=fe::AdvanceStaggeredCin(owner,token,{assembly.owner_id,assembly.accepted.base_epoch,
      assembly.attempt,cin.qualification_id,config.owner.fixed_dt,1.,true});
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if (report.status!=fe::NodalStatus::Ok) return false;
  report=owner.BorrowPrepared(token,&prepared);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  return report.status==fe::NodalStatus::Ok;
}
} // namespace qbat_mapped_test
