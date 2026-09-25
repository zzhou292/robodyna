// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../ShellGlobalLaw1Profile.h"
#include "../T3History.h"
#include "../T3OnePointCoefficients.h"
#include "../../ShellNodalStiffness.h"
#include "../../ShellSectionLaw.h"

namespace tl::fea::t3::mapped {
using NodalStiffness=shell_nodal_stiffness::Packet<3>;

// Complete C3UPDT3 adds full STI/STIR to each slot, without a 1/3 factor.
TL_T3_HD inline bool PackStiffness(const ForceDiagnostics& values,
    NodalStiffness& output) noexcept {
  if (!tl::math::Finite(values.translational_stiffness) || values.translational_stiffness<0 ||
      !tl::math::Finite(values.rotational_stiffness) || values.rotational_stiffness<0) return false;
  NodalStiffness next;
  for (unsigned slot=0;slot<3;++slot) {
    next.translation[slot]=values.translational_stiffness;
    next.rotation[slot]=values.rotational_stiffness;
  }
  output=next;
  return true;
}

TL_T3_HD inline bool InitialStiffness(const ReferenceData& reference,
    ShellSectionLaw law,NodalStiffness& output,const ShellGlobalLaw1Profile* global=nullptr) noexcept {
  const bool one_point=law==ShellSectionLaw::Law44Nip1;
  if (!reference.prepared || (!one_point && law!=ShellSectionLaw::LayeredLaw1Nip3 && law!=ShellSectionLaw::GlobalLaw1Npt0 &&
      law!=ShellSectionLaw::LayeredLaw44Nip3)) return false;
  if ((law==ShellSectionLaw::LayeredLaw1Nip3||law==ShellSectionLaw::GlobalLaw1Npt0) &&
      reference.input.placement!=ShellReferencePlacement::Centered) return false;
  double longest=0;
  if (!detail::SupportedGeometry(reference.input.position,longest)) return false;
  detail::GeometryWork geometry;
  if (detail::CurrentGeometry(reference.input.position,longest,geometry)!=Status::kSuccess) return false;
  detail::MaterialWork material;
  auto coefficient_input=reference.input;
  if(law==ShellSectionLaw::GlobalLaw1Npt0) {
    if(!global||!shell_global_law1::Valid(*global))return false;
    coefficient_input.thickness=shell_global_law1::T3CoefficientThickness(*global,reference.input.thickness,reference.input.thickness);
  }
  if (one_point) {
    if (!one_point_detail::PrepareCoefficients(reference.input,geometry.kinematics.area,
        reference.input.thickness,material)) return false;
  } else if (!detail::PrepareMaterial(coefficient_input,geometry.kinematics.area,material,
      reference.input.placement)) return false;
  ForceDiagnostics diagnostics;
  detail::StiffnessDiagnostics(geometry,material,diagnostics);
  const bool valid=one_point ? one_point_detail::ValidDiagnostics(diagnostics,1)
      : detail::ValidForceDiagnostics(diagnostics);
  return valid && PackStiffness(diagnostics,output);
}

TL_T3_HD inline bool AcceptedStiffness(const ForceTrial& accepted,
    NodalStiffness& output) noexcept {
  return PackStiffness(accepted.diagnostics,output);
}
} // namespace tl::fea::t3::mapped
