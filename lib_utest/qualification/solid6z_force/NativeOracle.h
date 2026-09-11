// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "native/NativeForce.h"
#include "../solid6z_reference/NativeOracle.h"
#include "lib_src/materials/law42/Types.h"
#include <array>

namespace solid6z_force_test {
struct NativeResult {
  std::array<double,98> geometry{};
  std::array<double,33> material{};
  std::array<double,21> history{};
  std::array<double,54> forces{};
  // Mode velocities, final modes, G, FCL, first and final EINT.
  std::array<double,28> stabilization{};
  int status = -1;
};
class NativeHistory {
 public:
  bool Initialize(const tl::fea::solid6z::ReferenceInput& input,
                  const tl::material::law42::Parameters& material) {
    const auto reference = solid6z_test::Native(input);
    if (reference.status != 0) return false;
    parameters_ = {material.mu_pa,material.poisson_ratio,
        material.density_kg_m3,material.tension_cutoff_pa};
    permutation_ = reference.permutation;
    for (unsigned n = 0; n < 6; ++n) {
      const auto x = input.position_m[permutation_[n]];
      original_[3*n] = x.x;
      original_[3*n+1] = x.y;
      original_[3*n+2] = x.z;
    }
    for (unsigned i = 0; i < 10; ++i) reference_[i] = reference.values[27+i];
    reference_[10] = reference.values[37];
    accepted_.fill(0);
    accepted_[6] = material.density_kg_m3;
    return true;
  }
  const std::array<double,21>& accepted() const { return accepted_; }
  void Prescribe(std::array<double,21> values) { accepted_ = values; }
  void Accept(const NativeResult& trial) { accepted_ = trial.history; }
  NativeResult Evaluate(const tl::fea::solid6z::PrescribedInterval& interval,
                        double damping = .1, double sound_speed_scale = 1) const {
    std::array<double,18> position{},velocity{};
    for (unsigned n = 0; n < 6; ++n) {
      const auto x = interval.position_endpoint_m[permutation_[n]];
      const auto v = interval.velocity_midpoint_m_s[permutation_[n]];
      position[3*n] = x.x; position[3*n+1] = x.y; position[3*n+2] = x.z;
      velocity[3*n] = v.x; velocity[3*n+1] = v.y; velocity[3*n+2] = v.z;
    }
    const double step[3]{interval.dt_s,damping,sound_speed_scale};
    NativeResult result;
    solid6z_force_native(parameters_.data(),original_.data(),reference_.data(),accepted_.data(),
        position.data(),velocity.data(),step,result.geometry.data(),result.material.data(),
        result.history.data(),result.forces.data(),result.stabilization.data(),&result.status);
    const auto native = result.forces;
    for (unsigned n = 0; n < 6; ++n) {
      for (unsigned k = 0; k < 3; ++k)
        result.forces[36+3*permutation_[n]+k] = native[36+3*n+k];
    }
    return result;
  }
 private:
  std::array<double,4> parameters_{};
  std::array<int,6> permutation_{};
  std::array<double,18> original_{};
  std::array<double,11> reference_{};
  std::array<double,21> accepted_{};
};
} // namespace solid6z_force_test
