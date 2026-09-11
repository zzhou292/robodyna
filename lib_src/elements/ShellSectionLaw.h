#pragma once
#include <cstddef>
#include <cstdint>

namespace tl::fea {
// Constitutive section role is independent of the LAW44 hardening curve kind.
enum class ShellSectionLaw : std::uint8_t {
  Unspecified=0,LayeredLaw44Nip3,LayeredLaw1Nip3,
  // Resolved section role, never a material declaration tag.
  Law44Nip1,
};
enum class ShellSectionFormulation : std::uint8_t {
  LayeredNip3,
  OneThicknessPoint,
};
struct ShellSectionCounts {
  std::size_t law44=0,law1=0;
  std::size_t law44_nip1=0; // Subset of law44, not another material total.
};
} // namespace tl::fea
