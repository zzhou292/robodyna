// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeChecks.h"
#include "../solid6z_force/Compare.h"

namespace solid_resident_test {
void CheckNative(const s::Result6z& result,const solid6z_force_test::NativeResult& expected,
    double storage_volume) {
  ASSERT_EQ(expected.status,0);
  const auto actual=Values(result);
  ASSERT_EQ(actual.size(),103u);
  // Reuse the qualified complete S6Z comparison scale. Geometry/local-force
  // channels are not retained by Batch; compare only its actual public fields.
  solid6z_force_test::Values packet;
  std::copy(expected.geometry.begin(),expected.geometry.end(),packet.geometry);
  std::copy(expected.forces.begin(),expected.forces.end(),packet.forces);
  std::copy_n(actual.data(),21,packet.history);
  std::copy_n(actual.data()+21,18,packet.forces+36);
  std::copy_n(actual.data()+39,33,packet.material);
  std::copy_n(actual.data()+72,26,packet.stabilization);
  packet.stabilization[26]=result.cache.material.history.internal_energy_density_j_m3+
      result.cache.stabilization.first_work_j/std::fmax(1e-20,storage_volume);
  packet.stabilization[27]=packet.stabilization[26]+
      result.cache.stabilization.second_work_j/std::fmax(1e-20,storage_volume);
  ASSERT_TRUE(solid6z_force_test::Agree(packet,expected));
  EXPECT_EQ(packet.stabilization[27],result.history.material.internal_energy_density_j_m3);
  EXPECT_EQ(result.cache.total_internal_work_increment_j,result.cache.material.internal_work_j+
      result.cache.stabilization.first_work_j+result.cache.stabilization.second_work_j);
  EXPECT_NEAR(result.cache.stiffness.translation_n_m,(1.0/3.0)*expected.material[32],
      3e-10*std::abs((1.0/3.0)*expected.material[32]));
  EXPECT_EQ(result.cache.stiffness.rotation_nm,0);
}
} // namespace solid_resident_test
