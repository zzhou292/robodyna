#pragma once
#include <cstddef>
#include <cstdint>

namespace tl::fea {
// Constitutive section role is independent of the LAW44 hardening curve kind.
enum class ShellSectionLaw : std::uint8_t {
  Unspecified=0,LayeredLaw44Nip3,LayeredLaw1Nip3,
  // Resolved section role, never a material declaration tag.
  Law44Nip1,
  // Four in-plane stations, each with the SAME source NIP1 section.
  // Resolved role only; never a material declaration tag.
  Law44QbatFourInPlane,
  // Explicit nonconstitutive execution role; no material-point parameters.
  RigidSkin,
  GlobalLaw1Npt0, // Analytic global integration: ordinary history, zero material points.
};
enum class ShellSectionFormulation : std::uint8_t {
  LayeredNip3,
  OneThicknessPoint,
  Nonconstitutive,
};
struct ShellSectionCounts {
  std::size_t law44=0,law1=0;
  std::size_t law44_nip1=0; // Subset of law44, not another material total.
  std::size_t law44_qbat=0; // Four in-plane points per parent; subset of law44.
  std::size_t rigid_skin=0; // Zero material points, not a LAW1/LAW44 subset.
  std::size_t law1_global_npt0=0; // Subset of law1, not another material total.
};
} // namespace tl::fea
