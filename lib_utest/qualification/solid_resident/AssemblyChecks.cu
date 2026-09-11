// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace solid_resident_test {
void Rig::CompareAssembly(const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view,
    const Results& result) {
  const auto n=config.owner.node_count;
  std::vector<double> expected(4*n),actual(4*n),couples(3*n),rotation(n);
  const auto add=[&](const auto& parent,const auto& cache,unsigned count) {
    for (unsigned k=0;k<count;++k) {
      const auto node=parent.domain_nodes[k];
      expected[node]+=cache.rhs_force_n[k].x;
      expected[n+node]+=cache.rhs_force_n[k].y;
      expected[2*n+node]+=cache.rhs_force_n[k].z;
      expected[3*n+node]+=cache.stiffness.translation_n_m;
    }
  };
  add(fixture.model.solid18()[0],result.a.cache,8);
  add(fixture.model.solid24()[0],result.b.cache,8);
  add(fixture.model.solid6z()[0],result.c.cache,6);
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(Good(owner.BorrowCinAssembly(token,&cin)));
  const double* fields[]{view.forces.force_x,view.forces.force_y,view.forces.force_z,cin.translational_stiffness};
  for (unsigned axis=0;axis<4;++axis)
    ASSERT_EQ(cudaMemcpyAsync(actual.data()+axis*n,fields[axis],n*sizeof(double),
        cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  const double* moment[]{view.forces.couple_x,view.forces.couple_y,view.forces.couple_z};
  for (unsigned axis=0;axis<3;++axis)
    ASSERT_EQ(cudaMemcpyAsync(couples.data()+axis*n,moment[axis],n*sizeof(double),
        cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(rotation.data(),cin.rotational_stiffness,n*sizeof(double),
      cudaMemcpyDeviceToHost,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  EXPECT_EQ(actual,expected);
  for (double value:couples) EXPECT_EQ(value,0);
  for (double value:rotation) EXPECT_EQ(value,0);
}
} // namespace solid_resident_test
