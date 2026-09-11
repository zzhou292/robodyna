// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cmath>
#include <cstdint>

#if defined(__CUDACC__)
#define TL_CIN_STRUCTURAL_HD __host__ __device__
#else
#define TL_CIN_STRUCTURAL_HD
#endif
namespace tl::fea {
enum class NodalCinStructuralProfile : std::uint8_t {
  Disabled,
  NativeOrdinaryRigidTrace
};
// Explicit local structural screen after CIN transfer, before motion. Ordinary
// DOFs use native DTNODA; rigid bodies use a current-frame scalar-surrogate trace
// bound. This is not an admission of the full nonlinear/contact/joint tangent.
// No mass scaling. The caller supplies its factor; no deck default is inferred.
struct NodalCinStructuralStep {
  NodalCinStructuralProfile profile = NodalCinStructuralProfile::Disabled;
  double factor = 0;
};
TL_CIN_STRUCTURAL_HD inline bool ValidCinStructuralStep(NodalCinStructuralStep policy) noexcept {
  if (policy.profile == NodalCinStructuralProfile::Disabled) return policy.factor == 0;
  return policy.profile == NodalCinStructuralProfile::NativeOrdinaryRigidTrace &&
      std::isfinite(policy.factor) && policy.factor > 0 && policy.factor <= 1;
}
} // namespace tl::fea
#undef TL_CIN_STRUCTURAL_HD
