// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <vector>

extern "C" void rear18_force_initial_native(const int*,const double*,const double*,const int*,
    const int*,const double*,const double*,const double*,const double*,const double*,const double*,
    double*,int*,double*,double*,double*,double*,double*,double*,int*);

namespace rear_startup_test {
rear_force_test::NativeResult NativeInitialize(const law::Material& material,
    const s::ReferenceInput& input, s::Vec3 velocity) {
  // All virgin volumes/frame/source permutations come from the independent
  // native starter, not from a production prepared reference or history.
  const auto initial = rear_force_test::NativeInitial(input);
  const int count = static_cast<int>(material.curve.count);
  std::vector<double> curve(2*(count+1));
  for (int i = 0; i < count; ++i) {
    curve[2*(i+1)] = material.curve.plastic_strain[i];
    curve[2*(i+1)+1] = material.curve.yield_stress_pa[i];
  }
  const auto& m = material.material;
  const double parameters[]{m.young_pa,m.poisson_ratio,m.density_kg_m3,m.rate_c_per_s,
      m.rate_p,m.cutoff_hz == 10000 ? 0 : m.cutoff_hz,270e6};
  const int units = static_cast<int>(m.native_units);
  double volumes[8], position[24], native_force[24];
  for (unsigned n = 0; n < 8; ++n) {
    volumes[n] = initial.point[20*n+18];
    const auto& x = input.position_m[initial.source_slot[n]];
    position[3*n] = x.x;
    position[3*n+1] = x.y;
    position[3*n+2] = x.z;
  }
  const double v[]{velocity.x,velocity.y,velocity.z};
  rear_force_test::NativeResult result;
  result.next = initial;
  rear18_force_initial_native(&count,curve.data(),parameters,&units,initial.local_nodes.data(),
      volumes,&initial.initial_center_volume,&initial.global[7],initial.saved.data(),position,v,
      result.next.point.data(),result.next.cursor.data(),result.next.global.data(),
      result.next.saved.data(),result.observation.data(),native_force,result.geometry.data(),
      result.diagnostics.data(),&result.status);
  if (result.status == 0) {
    for (unsigned n = 0; n < 8; ++n)
      for (unsigned k = 0; k < 3; ++k)
        result.force[3*initial.source_slot[n]+k] = native_force[3*n+k];
  }
  return result;
}
}  // namespace rear_startup_test
