// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../ShellLayeredSectionValues.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_MIXED_ACTIVITY_HD __host__ __device__
#else
#define TL_MIXED_ACTIVITY_HD
#endif

namespace tl::fea::qeph::mapped {
enum class MixedActivityError : std::uint32_t {
  None = 0, Elastic = 1, Plastic = 2, OnePoint = 3, Unsupported = 4
};
inline constexpr std::uint32_t MixedActivityKeyStride = 8;

TL_MIXED_ACTIVITY_HD inline MixedActivityError CheckMixedActivity(ShellSectionLaw law,
    const ShellBatchSectionState& plastic, const sections::ShellLayeredLaw1History& elastic) noexcept {
  using shell_batch_plasticity_detail::FiniteSection;
  if (law == ShellSectionLaw::LayeredLaw1Nip3)
    return FiniteSection(elastic) ? MixedActivityError::None : MixedActivityError::Elastic;
  if (law == ShellSectionLaw::LayeredLaw44Nip3)
    return FiniteSection(plastic) ? MixedActivityError::None : MixedActivityError::Plastic;
  if (law == ShellSectionLaw::Law44Nip1) return MixedActivityError::OnePoint;
  return law == ShellSectionLaw::RigidSkin ? MixedActivityError::None : MixedActivityError::Unsupported;
}

TL_MIXED_ACTIVITY_HD inline bool ValidMixedActivityRole(std::uint8_t role) noexcept {
  return role == static_cast<std::uint8_t>(ShellSectionLaw::LayeredLaw1Nip3) ||
      role == static_cast<std::uint8_t>(ShellSectionLaw::LayeredLaw44Nip3) ||
      role == static_cast<std::uint8_t>(ShellSectionLaw::RigidSkin);
}

// Parent dominates error kind, matching the original host parent traversal.
// Admitted QEPH count is at most 524288, well below this key's exact capacity.
TL_MIXED_ACTIVITY_HD inline std::uint32_t MixedActivityKey(std::uint32_t parent,
    MixedActivityError error) noexcept {
  return parent * MixedActivityKeyStride + static_cast<std::uint32_t>(error);
}
} // namespace tl::fea::qeph::mapped
#undef TL_MIXED_ACTIVITY_HD
