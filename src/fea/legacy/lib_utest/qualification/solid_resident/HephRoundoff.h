// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law42/Types.h"
#include <array>
#include <cmath>
#include <limits>

namespace solid_resident_test {
// Test-only error propagation for the loaded, near-identity resident trajectory.
// No runtime admission or general large-strain spectral error claim.
struct HephRoundoff {
  double stress_pa = 0;
  double force_n = 0;
  double material_work_j = 0;
  double material_energy_j_m3 = 0;
  double accepted_energy_j_m3 = 0;

  double Additional(unsigned field) const {
    if (field < 6 || (field >= 151 && field < 157) ||
        (field >= 160 && field < 168)) return stress_pa;
    if (field >= 22 && field < 46) return force_n;
    if (field == 7) return accepted_energy_j_m3;
    if (field == 158) return material_energy_j_m3;
    if (field == 181) return material_work_j;
    return 0;
  }
};

inline bool PrepareHephRoundoff(const tl::material::law42::Parameters& material,
    const std::array<double,187>& native, const std::array<double,21>& old,
    double storage_volume, double dt, const HephRoundoff& accepted,
    HephRoundoff& output) {
  for (double x : native) if (!std::isfinite(x)) return false;
  for (double x : old) if (!std::isfinite(x)) return false;
  if (!std::isfinite(storage_volume) || storage_volume <= 0 ||
      !std::isfinite(dt) || dt < 0 || material.bulk_pa <= 0 || material.mu_pa <= 0 ||
      !std::isfinite(material.bulk_pa + 2*material.mu_pa) || native[168] != 1 ||
      native[103] <= 0 || native[180] <= 0) return false;
  double strain_l1 = 0;
  for (unsigned k=173; k<179; ++k) strain_l1 += std::abs(native[k]);
  // This explicit domain contains the captured packet and the eight-step gate.
  // Outside it the caller must keep the previous strict comparison.
  if (strain_l1 > 1.0/16 || native[169] < .9 || native[169] > 1.1) return false;
  constexpr double epsilon = std::numeric_limits<double>::epsilon();
  HephRoundoff next;
  // C=I+strain, sqrt/product/log/exp and subtraction of O(mu) principal terms.
  // Same conservative operation scale as the qualified near-zero S6Z control.
  next.stress_pa = 128*epsilon*(material.bulk_pa + 2*material.mu_pa)*(1 + strain_l1);

  // SFINT3: each local RHS component is V times three stress-gradient terms;
  // SRROTA3 then multiplies the local vector by one frame row. Maxima over
  // every native slot/row remain valid after the source-slot permutation.
  double gradient_l1 = 0, frame_l1 = 0;
  for (unsigned n=0; n<4; ++n) {
    double sum = 0;
    for (unsigned k=0; k<3; ++k) sum += std::abs(native[105+4*k+n]);
    gradient_l1 = std::fmax(gradient_l1,sum);
  }
  for (unsigned n=0; n<3; ++n) {
    double sum = 0;
    for (unsigned k=0; k<3; ++k) sum += std::abs(native[46+3*n+k]);
    frame_l1 = std::fmax(frame_l1,sum);
  }
  next.force_n = next.stress_pa*native[103]*gradient_l1*frame_l1;

  // MULAW work: p=-(old_x+new_x+old_y+new_y+old_z+new_z)/3.
  // If each old/new stress differs by b_old/b_new, p differs by their sum;
  // each normal work term by twice that sum, and each shear by the sum.
  double normal_rates = 0, shear_rates = 0, stress_terms = 0;
  for (unsigned k=0; k<3; ++k) normal_rates += std::abs(native[145+k]);
  for (unsigned k=3; k<6; ++k) shear_rates += std::abs(native[145+k]);
  for (unsigned k=0; k<6; ++k) stress_terms += std::abs(old[k])+std::abs(native[k]);
  const double work_factor = .5*(std::abs(native[180]*dt)*(2*normal_rates+shear_rates)+
      std::abs(native[179]));
  next.material_work_j = work_factor*(accepted.stress_pa+next.stress_pa)+
      32*epsilon*work_factor*stress_terms;
  const double denominator = std::fmax(storage_volume,1e-20);
  next.material_energy_j_m3 = (accepted.accepted_energy_j_m3*storage_volume+
      next.material_work_j)/denominator+
      4*epsilon*(std::abs(old[7]*storage_volume)+std::abs(native[181]))/denominator;
  // HG work itself keeps its old independent comparison. Propagate that
  // existing allowance through its two energy additions, without widening it.
  const double hg_work_error = 3e-10*std::fmax(std::abs(native[184]),1e-20);
  next.accepted_energy_j_m3 = next.material_energy_j_m3+hg_work_error/denominator+
      4*epsilon*(std::abs(native[158])+std::abs(native[184])/denominator);
  for (double x : {next.stress_pa,next.force_n,next.material_work_j,
                  next.material_energy_j_m3,next.accepted_energy_j_m3})
    if (!std::isfinite(x) || x < 0) return false;
  output = next;
  return true;
}
} // namespace solid_resident_test
