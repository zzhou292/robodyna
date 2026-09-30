// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected NPTT1 LAW44 caller: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "Law44ZeroShearPlasticity.h"

namespace tl::material {
#if defined(__CUDACC__)
__host__ __device__
#endif
inline TabulatedShellPlasticityStatus UpdateLaw44MembranePlasticity(
    const TabulatedShellPlasticityParameters& parameters,
    const TabulatedShellPlasticityHistory& accepted,
    const TabulatedShellPlasticityInput& input,
    TabulatedShellPlasticityResult& output) noexcept {
  // CBADEF1/CBASTRA3 supply zero transverse increments. GS is exactly zero
  // from CNCOEF3, while SIGEPS44C still carries any supplied transverse stress.
  if (input.strain_increment[3]!=0 || input.strain_increment[4]!=0)
    return TabulatedShellPlasticityStatus::InvalidIncrement;
  return UpdateLaw44ZeroShearPlasticity(parameters,accepted,input,output);
}
} // namespace tl::material
