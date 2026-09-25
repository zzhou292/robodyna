// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "coefficients/MainShell.h"
#include "coefficients/MainSolid.h"
#include "coefficients/Nodal.h"
#include "coefficients/Pair.h"
#include "coefficients/UnitFactors.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateSiShellMainCoefficient(UnitScale units,
    const SiShellMainCoefficientInput& in, SiScalarCoefficient* output) {
  using namespace coefficient_detail;
  UnitFactors f;
  if (!output || !Make(units, f) || !Finite(in.scale) || !Nonnegative(in.element_thickness) ||
      !Nonnegative(in.property_thickness) || !Nonnegative(in.young)) return CoefficientStatus::InvalidInput;
  NativeShellMainCoefficientInput native;
  native.face = in.face; native.layout = in.layout; native.property_type = in.property_type;
  native.stack_material = in.stack_material; native.input_thickness_mode = in.input_thickness_mode;
  native.scale = in.scale; native.element_thickness = in.element_thickness / f.base.length;
  native.property_thickness = in.property_thickness / f.base.length; native.young = in.young / f.pressure;
  if (!Finite(native.element_thickness) || !Finite(native.property_thickness) || !Finite(native.young) ||
      (in.element_thickness != 0 && native.element_thickness == 0))
    return CoefficientStatus::NonfiniteResult;
  NativeScalarCoefficient result;
  const auto status = EvaluateNativeShellMainCoefficient(native, &result);
  if (status != CoefficientStatus::Ok) return status;
  const double value = result.value * f.base.stiffness;
  if (!Finite(value)) return CoefficientStatus::NonfiniteResult;
  *output = {value}; return CoefficientStatus::Ok;
}
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateSiSolidMainCoefficient(UnitScale units,
    const SiSolidMainCoefficientInput& in, SiSolidMainCoefficientResult* output) {
  using namespace coefficient_detail;
  UnitFactors f;
  if (!output || !Make(units, f) || !Finite(in.scale) || !Nonnegative(in.fill) ||
      !Positive(in.area) || !Positive(in.volume) || !Nonnegative(in.bulk) ||
      !Nonnegative(in.controlled_bulk)) return CoefficientStatus::InvalidInput;
  NativeSolidMainCoefficientInput native;
  native.face = in.face; native.layout = in.layout;
  native.incompressibility_control = in.incompressibility_control;
  native.scale = in.scale; native.fill = in.fill;
  native.area = in.area / f.area; native.volume = in.volume / f.volume;
  native.bulk = in.bulk / f.pressure; native.controlled_bulk = in.controlled_bulk / f.pressure;
  if (!Positive(native.area) || !Positive(native.volume) || !Finite(native.bulk) ||
      !Finite(native.controlled_bulk)) return CoefficientStatus::NonfiniteResult;
  NativeSolidMainCoefficientResult result;
  const auto status = EvaluateNativeSolidMainCoefficient(native, &result);
  if (status != CoefficientStatus::Ok) return status;
  SiSolidMainCoefficientResult si{result.stiffness * f.base.stiffness,
      result.characteristic_length * f.base.length};
  if (!Finite(si.stiffness) || !Finite(si.characteristic_length)) return CoefficientStatus::NonfiniteResult;
  *output = si; return CoefficientStatus::Ok;
}
TL_MATH_HOST_DEVICE inline CoefficientStatus FinalizeSiNodalCoefficient(UnitScale units,
    const SiAccumulatedNodalCoefficients& in, SiNodalCoefficientResult* output) {
  using namespace coefficient_detail;
  UnitFactors f;
  if (!output || !Make(units, f) || !Nonnegative(in.volume) || !Nonnegative(in.bulk_volume) ||
      !Nonnegative(in.young_thickness_sum) || !Nonnegative(in.existing_stiffness) ||
      in.shell_incidence_count < 0) return CoefficientStatus::InvalidInput;
  // Pressure*volume is energy; normalized_bulk is pressure, not the same field.
  NativeAccumulatedNodalCoefficients native{in.volume / f.volume,
      in.bulk_volume / f.base.energy, in.young_thickness_sum / f.base.stiffness,
      in.shell_incidence_count, in.existing_stiffness / f.base.stiffness};
  if (!Finite(native.volume) || !Finite(native.bulk_volume) || !Finite(native.young_thickness_sum) ||
      !Finite(native.existing_stiffness)) return CoefficientStatus::NonfiniteResult;
  NativeNodalCoefficientResult result;
  const auto status = FinalizeNativeNodalCoefficient(native, &result);
  if (status != CoefficientStatus::Ok) return status;
  SiNodalCoefficientResult si{result.normalized_bulk * f.pressure, result.stiffness * f.base.stiffness};
  if (!Finite(si.normalized_bulk) || !Finite(si.stiffness)) return CoefficientStatus::NonfiniteResult;
  *output = si; return CoefficientStatus::Ok;
}
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateSiSecondaryCoefficient(UnitScale units,
    const SiSecondaryCoefficientInput& in, SiScalarCoefficient* output) {
  using namespace coefficient_detail;
  UnitFactors f;
  if (!output || !Make(units, f) || !Finite(in.existing) || !Finite(in.scale) ||
      (in.existing != 0 && !Nonnegative(in.global_stiffness))) return CoefficientStatus::InvalidInput;
  NativeSecondaryCoefficientInput native{in.existing / f.base.stiffness,
      in.existing == 0 ? 0 : in.global_stiffness / f.base.stiffness, in.scale};
  if (!Finite(native.existing) || !Finite(native.global_stiffness) ||
      (in.existing != 0 && native.existing == 0)) return CoefficientStatus::NonfiniteResult;
  NativeScalarCoefficient result;
  const auto status = EvaluateNativeSecondaryCoefficient(native, &result);
  if (status != CoefficientStatus::Ok) return status;
  const double value = result.value * f.base.stiffness;
  if (!Finite(value)) return CoefficientStatus::NonfiniteResult;
  *output = {value}; return CoefficientStatus::Ok;
}
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateSiPairCoefficient(UnitScale units,
    PairCoefficientProfile profile, const SiPairCoefficientInput& in, SiScalarCoefficient* output) {
  using namespace coefficient_detail;
  UnitFactors f;
  if (!output || !Make(units, f) || !Finite(in.main) || !Finite(in.secondary) ||
      !Nonnegative(in.minimum) || !Nonnegative(in.maximum) || in.minimum > in.maximum)
    return CoefficientStatus::InvalidInput;
  NativePairCoefficientInput native{in.main / f.base.stiffness, in.secondary / f.base.stiffness,
      in.minimum / f.base.stiffness, in.maximum / f.base.stiffness};
  if (!Finite(native.main) || !Finite(native.secondary) || !Finite(native.minimum) || !Finite(native.maximum))
    return CoefficientStatus::NonfiniteResult;
  NativeScalarCoefficient result;
  const auto status = EvaluateNativePairCoefficient(profile, native, &result);
  if (status != CoefficientStatus::Ok) return status;
  const double value = result.value * f.base.stiffness;
  if (!Finite(value)) return CoefficientStatus::NonfiniteResult;
  *output = {value}; return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
