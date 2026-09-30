// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../ShellGlobalLaw1Profile.h"
#include "../QephCurrentFrame.h"
#include "../QephStiffnessDiagnostics.h"
#include "../../ShellNodalStiffness.h"
#include "../../ShellSectionLaw.h"

namespace tl::fea::qeph::mapped {
using NodalStiffness=shell_nodal_stiffness::Packet<4>;

TL_QEPH_HD inline bool PackStiffness(const ForceDiagnostics& values,
    const double (&factor)[2],NodalStiffness& output) noexcept {
  if (!tl::math::Finite(values.translational_stiffness) || values.translational_stiffness<0 ||
      !tl::math::Finite(values.rotational_stiffness) || values.rotational_stiffness<0 ||
      !detail::Positive(factor[0]) || !detail::Positive(factor[1])) return false;
  NodalStiffness next;
  for (unsigned slot=0;slot<4;++slot) {
    next.translation[slot]=values.translational_stiffness*factor[slot%2];
    next.rotation[slot]=values.rotational_stiffness*factor[slot%2];
    if (!tl::math::Finite(next.translation[slot]) || !tl::math::Finite(next.rotation[slot])) return false;
  }
  output=next;
  return true;
}

// Coefficient-only initial packet. No material update, history advance or
// completed force diagnostic is created. Initial CINMAS is not substituted.
TL_QEPH_HD inline bool InitialStiffness(const ReferenceData& reference,
    ShellSectionLaw law,NodalStiffness& output,const ShellGlobalLaw1Profile* global=nullptr) noexcept {
  if (!reference.prepared || (law!=ShellSectionLaw::LayeredLaw1Nip3 && law!=ShellSectionLaw::GlobalLaw1Npt0 &&
      law!=ShellSectionLaw::LayeredLaw44Nip3)) return false;
  if ((law==ShellSectionLaw::LayeredLaw1Nip3||law==ShellSectionLaw::GlobalLaw1Npt0) &&
      reference.input.placement!=ShellReferencePlacement::Centered) return false;
  PrescribedInterval coordinates;
  for (unsigned slot=0;slot<4;++slot) coordinates.position_endpoint[slot]=reference.input.position[slot];
  detail::GeometryWork geometry;
  if (detail::CurrentGeometry(coordinates,geometry)!=Status::kSuccess) return false;
  detail::MaterialWork material;
  auto coefficient_input=reference.input;
  if(law==ShellSectionLaw::GlobalLaw1Npt0) {
    if(!global||!shell_global_law1::Valid(*global))return false;
    coefficient_input.thickness=shell_global_law1::QephCoefficientThickness(*global,reference.input.thickness,reference.input.thickness);
  }
  if (!detail::PrepareMaterial(coefficient_input,geometry.values.area,0,material,
      reference.input.placement)) return false;
  ForceDiagnostics diagnostics;
  detail::StiffnessDiagnostics(geometry,material,diagnostics);
  return detail::ValidForceDiagnostics(diagnostics) &&
      PackStiffness(diagnostics,geometry.values.nodal_factors,output);
}

TL_QEPH_HD inline bool AcceptedStiffness(const ForceTrial& accepted,
    NodalStiffness& output) noexcept {
  return PackStiffness(accepted.diagnostics,accepted.kinematics.nodal_factors,output);
}
} // namespace tl::fea::qeph::mapped
