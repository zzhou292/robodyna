// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law44/solid/Prepare.h"

namespace tl::material::law44::beam {
// Reuse the qualified reader/curve preparation. Only E, G, C/P, the native
// finite cap and the tabulated curve are consumed by the three-stress leaf.
using Parameters = solid::Parameters;
using Material = solid::Material;
using Curve = solid::Curve;
using Status = solid::Status;
namespace detail {
TL_LAW44_SOLID_HD inline bool ParametersValid(const Parameters& p) noexcept {
  return p.material.hardening == solid::HardeningKind::Tabulated && solid::detail::ParametersValid(p);
}
} // namespace detail
TL_LAW44_SOLID_HD inline Status Prepare(Material material, Curve curve, Parameters& output) noexcept {
  if (material.hardening != solid::HardeningKind::Tabulated) return Status::InvalidParameters;
  return solid::Prepare(material, curve, output);
}
struct History {
  double stress_pa[3]{};  // XX, XY, XZ in the current beam section frame.
  double plastic_strain = 0;
  std::uint32_t curve_cursor = 0;
};
struct Input {
  // Actual MULAW_IB increments, including its preceding shear division.
  double strain_increment[3]{};
  double total_axial_strain = 0;
  double filtered_neutral_rate_per_s = 0;
};
struct Result {
  History history{};
  double plastic_increment = 0;
  double yield_stress_pa = 0;
  double tangent_factor = 1;  // SIGEPS44PI leaves ETSE=1.
};
} // namespace tl::material::law44::beam
