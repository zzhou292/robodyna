// SPDX-License-Identifier: AGPL-3.0-or-later
// I25GAPM shell-over-solid stiffness and asymmetric shell partner assignment.
#pragma once
#include "MainShell.h"
#include "MainSolid.h"
#include "lib_src/math/ScalarBits.h"
namespace tlfea::contact::radioss_type25 {
// Resolved arithmetic only: these operands do not authenticate membership,
// orientation, INSOL3D/VOLINT geometry, material ownership or main identities.
struct NativeCoatedMainCoefficientInput {
  NativeShellMainCoefficientInput shell; // face must be Coating.
  NativeSolidMainCoefficientInput solid; // exterior EightSlot support only.
};
struct NativeCoatedMainCoefficientResult {
  double primary_stiffness = 0;
  double partner_stiffness = 0;
  double solid_characteristic_length = 0; // GAP_N before later nodal gap distribution.
};
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeCoatedMainCoefficient(
    const NativeCoatedMainCoefficientInput& in, NativeCoatedMainCoefficientResult* output) {
  if (!output) return CoefficientStatus::InvalidInput;
  if (in.solid.layout != SolidLayout::EightSlot)
    return CoefficientStatus::UnsupportedProfile;
  // I25GAPM uses one SLSFAC for both contributions, including signed zero.
  if (!tl::math::SameScalarBits(in.shell.scale, in.solid.scale))
    return CoefficientStatus::InvalidInput;
  double shell;
  auto status = coefficient_detail::ShellContribution(in.shell, MainFaceKind::Coating, &shell);
  if (status != CoefficientStatus::Ok) return status;
  NativeSolidMainCoefficientResult solid;
  status = EvaluateNativeSolidMainCoefficient(in.solid, &solid);
  if (status != CoefficientStatus::Ok) return status;
  // GNU native MAX selects the last equal operand. This preserves STC's zero
  // sign; do not change the pre-existing ordinary-face compatibility helper.
  const double primary = solid.stiffness > shell ? solid.stiffness : shell;
  *output = {primary, shell, solid.characteristic_length};
  return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
