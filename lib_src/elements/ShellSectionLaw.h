#pragma once
#include <cstddef>
#include <cstdint>

namespace tl::fea {
// Constitutive section role is independent of the LAW44 hardening curve kind.
enum class ShellSectionLaw : std::uint8_t {
  Unspecified=0,LayeredLaw44Nip3,LayeredLaw1Nip3,
};
struct ShellSectionCounts {
  std::size_t law44=0,law1=0;
};
} // namespace tl::fea
