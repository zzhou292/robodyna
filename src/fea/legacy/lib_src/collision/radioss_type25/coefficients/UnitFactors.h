// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../UnitConversions.h"
#include "Common.h"
namespace tlfea::contact::radioss_type25::coefficient_detail {
struct UnitFactors { units_detail::Factors base; double area, volume, pressure; };
TL_MATH_HOST_DEVICE inline bool Make(UnitScale units, UnitFactors& f) {
  if (!units_detail::Make(units, f.base)) return false;
  f.area = f.base.length * f.base.length;
  f.volume = f.area * f.base.length;
  if (!Positive(f.area) || !Positive(f.volume)) return false;
  f.pressure = f.base.force / f.area;
  return Positive(f.pressure);
}
} // namespace tlfea::contact::radioss_type25::coefficient_detail
