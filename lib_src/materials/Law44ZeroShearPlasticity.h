// SPDX-License-Identifier: AGPL-3.0-or-later
// Native SIGEPS44C with GS=0; OpenRadioss (C) 2026 Siemens.
#pragma once
#include "TabulatedShellPlasticity.h"

namespace tl::material {
#if defined(__CUDACC__)
__host__ __device__
#endif
inline TabulatedShellPlasticityStatus UpdateLaw44ZeroShearPlasticity(
    const TabulatedShellPlasticityParameters& parameters,
    const TabulatedShellPlasticityHistory& accepted,
    const TabulatedShellPlasticityInput& input,
    TabulatedShellPlasticityResult& output) noexcept {
  // Zero transverse stiffness does not imply zero kinematic increment.
  // Preserve all five native increments and any carried transverse stress.
  return tabulated_shell_detail::UpdateLaw44PlasticityImpl(parameters,accepted,input,output,true);
}
} // namespace tl::material
