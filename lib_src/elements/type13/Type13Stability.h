// SPDX-License-Identifier: AGPL-3.0-or-later
// R4DEF3 six-channel Ileng1 coefficients and R2LEN3 unscaled scalar dt.
#pragma once
#include "Type13RecurrenceChecks.h"

namespace tl::fea::type13 {
TL_TYPE13_HD inline Status CriticalStep(
    const Property& property, const Reference& reference,
    double current_length_native, Stability& output) {
  detail::UnitFactors units;
  if (!detail::ValidReference(property, reference) ||
      !detail::ResolveUnits(property.units(), units) ||
      !detail::Positive(current_length_native)) {
    return Status::InvalidInput;
  }
  const double original_length = reference.length_native;
  const double length = current_length_native;
  const double axial = property.channel(0).native_stiffness;
  const double shear_y = property.channel(1).native_stiffness;
  const double shear_z = property.channel(2).native_stiffness;
  const double torsion = property.channel(3).native_stiffness;
  const double bending_y = property.channel(4).native_stiffness;
  const double bending_z = property.channel(5).native_stiffness;
  const double maximum_translation = ::fmax(::fmax(axial, shear_y), shear_z);
  const double shear_arm = ::fmax(shear_y, shear_z) * length * length;
  double rotation = ::fmax(::fmax(torsion, bending_y), bending_z) + shear_arm;
  double mass = property.mass_per_length() * original_length;
  double inertia = property.inertia_per_length() * original_length;
  double translation = maximum_translation / original_length;
  rotation = rotation / original_length;
  if (!detail::Positive(mass) || !detail::Positive(inertia) ||
      !detail::Positive(translation) || !detail::Positive(rotation)) {
    return Status::NonfiniteResult;
  }

  Stability next;
  next.translation_stiffness_N_per_m =
      translation * units.force / property.units().length_to_m;
  next.rotation_stiffness_Nm_per_rad =
      rotation * units.force * property.units().length_to_m;
  // Resolved source damping is zero. Keep native R2LEN3 evaluation order.
  if (0 + translation < 1e-15) {
    mass = 1;
  }
  translation = ::fmax(1e-15, translation);
  const double translation_dt =
      mass / ::fmax(1e-15, ::sqrt(0 * 0 + mass * translation) + 0);
  if (0 + rotation < 1e-15) {
    inertia = 1;
  }
  rotation = ::fmax(1e-15, rotation);
  const double rotation_dt =
      inertia / ::fmax(1e-15, ::sqrt(0 * 0 + inertia * rotation) + 0);
  next.critical_dt_s =
      ::fmin(translation_dt, rotation_dt) * property.units().time_to_s;
  if (!detail::Positive(translation_dt) || !detail::Positive(rotation_dt) ||
      !detail::Positive(next.critical_dt_s) ||
      !detail::Positive(next.translation_stiffness_N_per_m) ||
      !detail::Positive(next.rotation_stiffness_Nm_per_rad)) {
    return Status::NonfiniteResult;
  }
  output = next;
  return Status::Success;
}
} // namespace tl::fea::type13
