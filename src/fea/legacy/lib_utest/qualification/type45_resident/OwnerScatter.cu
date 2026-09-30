// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace type45_resident_test {
bool JointScatter(Rig& rig,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view) {
  std::array<joint::Result,3> cache;joint::BatchDiagnostics d;
  if(!rig.CopyAccepted(cache,d)) return false;
  fe::NodalCinAssemblyView cin;
  if(!Good(rig.physical.owner.BorrowCinAssembly(token,&cin))) return false;
  const double* arrays[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,
    view.forces.couple_x,view.forces.couple_y,view.forces.couple_z,
    cin.translational_stiffness,cin.rotational_stiffness};
  const auto count=view.accepted.node_count;
  std::vector<double> expected(8*count),actual(8*count);
  for(unsigned k=0;k<8;++k)
    if(cudaMemcpyAsync(expected.data()+k*count,arrays[k],count*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess)
      return false;
  if(cudaStreamSynchronize(view.stream)!=cudaSuccess) return false;
  for(std::size_t j=0;j<cache.size();++j) for(unsigned e=0;e<2;++e) {
    const auto n=rig.model.joints()[j].domain_nodes[e];const auto& c=cache[j].endpoint[e];
    const double values[]{c.force_n.x,c.force_n.y,c.force_n.z,c.couple_nm.x,c.couple_nm.y,c.couple_nm.z,
      c.translational_stiffness_n_m,c.rotational_stiffness_nm};
    for(unsigned k=0;k<8;++k) expected[k*count+n]+=values[k];
  }
  if(!Good(rig.joints.AssembleAccepted(rig.physical.owner,token,view))) return false;
  for(unsigned k=0;k<8;++k)
    if(cudaMemcpyAsync(actual.data()+k*count,arrays[k],count*sizeof(double),cudaMemcpyDeviceToHost,view.stream)!=cudaSuccess)
      return false;
  if(cudaStreamSynchronize(view.stream)!=cudaSuccess) return false;
  for(std::size_t k=0;k<actual.size();++k) EXPECT_EQ(common::Bits(actual[k]),common::Bits(expected[k]))<<k;
  return true;
}
} // namespace type45_resident_test
