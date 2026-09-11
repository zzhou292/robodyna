// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "lib_utest/qualification/solid18_law44_reference/NativeOracle.h"
#include <stdexcept>
#include <vector>

extern "C" void rear18_force_native(const int*,const double*,const double*,const int*,
    const int*,const double*,const int*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,double*,int*,double*,
    double*,double*,double*,double*,double*,int*);

namespace rear_force_test {
NativeState NativeInitial(const s::ReferenceInput& input) {
  const auto reference = rear18_test::Native(input);
  if (reference.status != 0) throw std::runtime_error("Native selected starter rejected input");
  NativeState state;
  state.source_slot = reference.permutation;
  state.initial_center_volume = reference.values[134];
  state.global[7] = reference.values[147];
  for (unsigned ip = 0; ip < 8; ++ip) {
    state.point[20*ip+16] = input.density_kg_m3;
    state.point[20*ip+17] = reference.values[63+10*ip];
    state.point[20*ip+18] = reference.values[63+10*ip];
  }
  for (unsigned n = 0; n < 7; ++n) {
    for (unsigned k = 0; k < 3; ++k) {
      state.saved[3*n+k] = reference.values[9+3*n+k]-reference.values[30+k];
    }
  }
  for (unsigned n = 0; n < 8; ++n) {
    const unsigned source = state.source_slot[n];
    unsigned first = 0;
    while (input.source_node_id[first] != input.source_node_id[source]) ++first;
    state.local_nodes[n] = static_cast<int>(first+1);
  }
  return state;
}

NativeResult Native(const law::Material& material, const NativeState& accepted,
                    const s::PrescribedInterval& interval) {
  const auto count = material.curve.count;
  if (count < 2 || count > 1024) throw std::invalid_argument("Native curve count");
  // Complete VINTER uses a leading unused pair, then the original points.
  std::vector<double> curve(2*(count+1));
  for (unsigned i = 0; i < count; ++i) {
    curve[2*(i+1)] = material.curve.plastic_strain[i];
    curve[2*(i+1)+1] = material.curve.yield_stress_pa[i];
  }
  double x[24], velocity[24], native_force[24];
  for (unsigned n = 0; n < 8; ++n) {
    const int source = accepted.source_slot[n];
    if (source < 0 || source >= 8) throw std::invalid_argument("Native source slot");
    const auto& position = interval.position_endpoint_m[source];
    const auto& v = interval.velocity_midpoint_m_s[source];
    x[3*n] = position.x;
    x[3*n+1] = position.y;
    x[3*n+2] = position.z;
    velocity[3*n] = v.x;
    velocity[3*n+1] = v.y;
    velocity[3*n+2] = v.z;
  }
  NativeResult result;
  result.next = accepted;
  const int npts = static_cast<int>(count);
  const auto& m = material.material;
  const double parameters[]{m.young_pa,m.poisson_ratio,m.density_kg_m3,m.rate_c_per_s,
      m.rate_p,m.cutoff_hz == 10000 ? 0 : m.cutoff_hz,270e6};
  const int units = static_cast<int>(m.native_units);
  rear18_force_native(&npts,curve.data(),parameters,&units,accepted.local_nodes.data(),
      accepted.point.data(),accepted.cursor.data(),accepted.global.data(),accepted.saved.data(),
      &accepted.initial_center_volume,x,velocity,&interval.base_time_s,&interval.dt_s,
      result.next.point.data(),result.next.cursor.data(),result.next.global.data(),
      result.next.saved.data(),result.observation.data(),native_force,result.geometry.data(),
      result.diagnostics.data(),&result.status);
  if (result.status == 0) {
    for (unsigned n = 0; n < 8; ++n) {
      for (unsigned k = 0; k < 3; ++k) {
        result.force[3*accepted.source_slot[n]+k] = native_force[3*n+k];
      }
    }
  }
  return result;
}
} // namespace rear_force_test
