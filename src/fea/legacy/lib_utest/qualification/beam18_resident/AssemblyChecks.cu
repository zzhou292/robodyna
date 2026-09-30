// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace beam18_resident_test {
void CheckAssembly(Rig& rig, const fe::NodalTrialToken& token, const fe::NodalAssemblyView& assembly,
    const Results& accepted) {
  fe::NodalCinAssemblyView cin;
  ASSERT_TRUE(Good(rig.owner.BorrowCinAssembly(token, &cin)));
  const std::size_t count = assembly.forces.node_count;
  std::array<std::vector<double>, 8> expected, actual;
  for (auto& row : expected) row.resize(count);
  for (auto& row : actual) row.resize(count);
  for (std::size_t p = 0; p < accepted.size(); ++p) {
    for (unsigned n = 0; n < 2; ++n) {
      const auto node = rig.fixture.model.parents()[p].domain_nodes[n];
      const auto& f = accepted[p].rhs_force_n[n]; const auto& m = accepted[p].rhs_couple_nm[n];
      const double values[]{f.x, f.y, f.z, m.x, m.y, m.z,
          accepted[p].diagnostics.translation_stiffness_n_m, accepted[p].diagnostics.rotation_stiffness_nm};
      for (unsigned k = 0; k < 8; ++k) expected[k][node] += values[k];
    }
  }
  const double* source[]{assembly.forces.force_x, assembly.forces.force_y, assembly.forces.force_z,
      assembly.forces.couple_x, assembly.forces.couple_y, assembly.forces.couple_z,
      cin.translational_stiffness, cin.rotational_stiffness};
  for (unsigned k = 0; k < 8; ++k)
    ASSERT_EQ(cudaMemcpyAsync(actual[k].data(), source[k], count*sizeof(double), cudaMemcpyDeviceToHost, cin.stream), cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(cin.stream), cudaSuccess);
  for (unsigned k = 0; k < 8; ++k) for (std::size_t n = 0; n < count; ++n) {
    SCOPED_TRACE(k);
    SCOPED_TRACE(n);
    EXPECT_EQ(actual[k][n], expected[k][n]);
  }
}
} // namespace beam18_resident_test
