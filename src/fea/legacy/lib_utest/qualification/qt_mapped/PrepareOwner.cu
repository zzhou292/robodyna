// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qt_mapped_test {
bool PrepareOwner(Fixture& fixture,fe::FENodalState& owner,const fe::NodalTrialToken& token,
    fe::NodalAssemblyView& assembly,fe::NodalPreparedView& prepared) {
  fe::NodalCinAssemblyView cin;
  if (owner.BorrowCinAssembly(token,&cin).status!=fe::NodalStatus::Ok) return false;
  std::vector<double> stiffness(cin.node_count);
  if (cudaMemcpyAsync(stiffness.data(),cin.translational_stiffness,stiffness.size()*sizeof(double),
      cudaMemcpyDeviceToHost,cin.stream)!=cudaSuccess || cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  for (std::size_t row=0;row<fixture.mechanics.cin_model.rows().count;++row) {
    const auto& r=fixture.mechanics.cin_model.rows().data[row];
    for (auto node:r.master_domain_nodes) stiffness[node]+=1;
    stiffness[r.secondary_domain_node]+=1;
  }
  if (cudaMemcpyAsync(cin.translational_stiffness,stiffness.data(),stiffness.size()*sizeof(double),
      cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess) return false;
  std::vector<std::uint8_t> activity(cin.witness_count,1);
  if (cudaMemcpyAsync(cin.witness_activity,activity.data(),activity.size(),cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess) return false;
  const double load=1000;
  const auto node=fixture.mechanics.domain.Find(12);
  if (cudaMemcpyAsync(assembly.forces.force_x+node,&load,sizeof(load),cudaMemcpyHostToDevice,cin.stream)!=cudaSuccess ||
      cudaStreamSynchronize(cin.stream)!=cudaSuccess) return false;
  auto report=owner.SealAssembly(token);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if (report.status!=fe::NodalStatus::Ok) return false;
  report=fe::AdvanceStaggeredCin(owner,token,{assembly.owner_id,assembly.accepted.base_epoch,
      assembly.attempt,cin.qualification_id,owner.accepted().fixed_dt,1.,true});
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  if (report.status!=fe::NodalStatus::Ok) return false;
  report=owner.BorrowPrepared(token,&prepared);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;
  return report.status==fe::NodalStatus::Ok;
}
} // namespace qt_mapped_test
