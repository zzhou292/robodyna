// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
namespace type25_coefficient_test {
// Independent dimensional expectations. No production conversion helper used.
struct Dimensions { double length, area, volume, pressure, stiffness, energy; };
inline Dimensions DimensionsOf(n::UnitScale u) {
  const double time2 = u.time_s * u.time_s;
  return {u.length_m, u.length_m * u.length_m, u.length_m * u.length_m * u.length_m,
      u.mass_kg / (u.length_m * time2), u.mass_kg / time2,
      u.mass_kg * u.length_m * u.length_m / time2};
}
struct SiPacket {
  Kind kind;
  n::UnitScale units;
  n::SiShellMainCoefficientInput shell;
  n::SiSolidMainCoefficientInput solid;
  n::SiAccumulatedNodalCoefficients nodal;
  n::SiSecondaryCoefficientInput secondary;
  n::SiPairCoefficientInput pair;
};
inline SiPacket ToSi(const Packet& p, n::UnitScale units) {
  SiPacket out; out.kind = p.kind; out.units = units;
  const auto d = DimensionsOf(units);
  if (p.kind == Kind::Shell) {
    const auto& in = p.shell; auto& si = out.shell;
    si.face = in.face; si.layout = in.layout; si.property_type = in.property_type;
    si.stack_material = in.stack_material; si.input_thickness_mode = in.input_thickness_mode;
    si.scale = in.scale; si.element_thickness = in.element_thickness * d.length;
    si.property_thickness = in.property_thickness * d.length; si.young = in.young * d.pressure;
  } else if (p.kind == Kind::Solid) {
    const auto& in = p.solid; auto& si = out.solid;
    si.face = in.face; si.layout = in.layout; si.incompressibility_control = in.incompressibility_control;
    si.scale = in.scale; si.fill = in.fill; si.area = in.area * d.area;
    si.volume = in.volume * d.volume; si.bulk = in.bulk * d.pressure;
    si.controlled_bulk = in.controlled_bulk * d.pressure;
  } else if (p.kind == Kind::Nodal) {
    const auto& in = p.nodal;
    out.nodal = {in.volume * d.volume, in.bulk_volume * d.energy,
        in.young_thickness_sum * d.stiffness, in.shell_incidence_count,
        in.existing_stiffness * d.stiffness};
  } else if (p.kind == Kind::Secondary) {
    out.secondary = {p.secondary.existing * d.stiffness, p.secondary.global_stiffness * d.stiffness,
        p.secondary.scale};
  } else {
    out.pair = {p.pair.main * d.stiffness, p.pair.secondary * d.stiffness,
        p.pair.minimum * d.stiffness, p.pair.maximum * d.stiffness};
  }
  return out;
}
TL_MATH_HOST_DEVICE inline Result Evaluate(const SiPacket& p) {
  Result result;
  if (p.kind == Kind::Shell) {
    n::SiScalarCoefficient out;
    result.status = n::EvaluateSiShellMainCoefficient(p.units, p.shell, &out); result.first = out.value;
  } else if (p.kind == Kind::Solid) {
    n::SiSolidMainCoefficientResult out;
    result.status = n::EvaluateSiSolidMainCoefficient(p.units, p.solid, &out);
    result.first = out.stiffness; result.second = out.characteristic_length;
  } else if (p.kind == Kind::Nodal) {
    n::SiNodalCoefficientResult out;
    result.status = n::FinalizeSiNodalCoefficient(p.units, p.nodal, &out);
    result.first = out.normalized_bulk; result.second = out.stiffness;
  } else if (p.kind == Kind::Secondary) {
    n::SiScalarCoefficient out;
    result.status = n::EvaluateSiSecondaryCoefficient(p.units, p.secondary, &out); result.first = out.value;
  } else {
    n::SiScalarCoefficient out;
    result.status = n::EvaluateSiPairCoefficient(p.units, {4, 0}, p.pair, &out); result.first = out.value;
  }
  return result;
}
inline Result ScaledReference(const Packet& p, n::UnitScale units) {
  auto result = Reference(p); const auto d = DimensionsOf(units);
  if (p.kind == Kind::Nodal) { result.first *= d.pressure; result.second *= d.stiffness; }
  else { result.first *= d.stiffness; if (p.kind == Kind::Solid) result.second *= d.length; }
  return result;
}
} // namespace type25_coefficient_test
