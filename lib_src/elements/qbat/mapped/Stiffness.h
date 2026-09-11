// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected CNDT3/CUPDTN3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "../QbatBatchArena.h"
#include "../QbatForceCoefficients.h"
#include "../QbatGeometry.h"

namespace tl::fea::qbat::mapped {
struct NodalStiffness {
  double translation[4]{};
  double rotation[4]{};
};
// Separate coefficient packet; virgin force/history/diagnostic availability is
// unchanged. No dt-zero material call and no reference CINMAS STIR substitution.
TL_QBAT_HD inline bool AcceptedStiffness(const batch_detail::Element& element,
    const BatchResult& accepted,NodalStiffness& output) noexcept {
  double stiffness=accepted.diagnostics.translation_stiffness_n_m;
  double rotational=accepted.diagnostics.rotation_stiffness_nm;
  double factor[2]{accepted.kinematics.nodal_factor[0],accepted.kinematics.nodal_factor[1]};
  if (!accepted.stamp.sample_index) {
    Kinematics initial;
    CurrentInput current;
    PrescribedInterval coordinates;
    for (unsigned slot=0;slot<4;++slot) {
      current.position_m[slot]=element.reference.input().quadrilateral.position[slot];
      coordinates.position_endpoint[slot]=current.position_m[slot];
    }
    if (EvaluateGeometry(element.reference,current,initial.geometry)!=Status::kSuccess ||
        !detail::CurrentLength(coordinates,initial)) return false;
    ForceDiagnostics coefficient;
    if (!detail::ForceCoefficients(element.reference,element.material,accepted.history,
        accepted.history.element_active,initial,coefficient)) return false;
    stiffness=coefficient.translation_stiffness_n_m;
    rotational=coefficient.rotation_stiffness_nm;
    factor[0]=initial.nodal_factor[0];
    factor[1]=initial.nodal_factor[1];
  }
  if (!tl::math::Finite(stiffness) || stiffness<0 || rotational!=0 ||
      !detail::Positive(factor[0]) || !detail::Positive(factor[1])) return false;
  NodalStiffness next;
  for (unsigned slot=0;slot<4;++slot) {
    const unsigned diagonal=slot%2;
    next.translation[slot]=stiffness*factor[diagonal];
    next.rotation[slot]=rotational*factor[diagonal];
    if (!tl::math::Finite(next.translation[slot]) || !tl::math::Finite(next.rotation[slot])) return false;
  }
  output=next;
  return true;
}
// Four distinct mapped cell slots; all additions are checked before publication.
// Parents may share physical nodes, and retain their distinct additive terms.
TL_QBAT_HD inline bool AddStiffness(const std::size_t (&nodes)[4],const NodalStiffness& value,
    double* translation,double* rotation,std::size_t count) noexcept {
  if (!translation || !rotation || translation==rotation) return false;
  double next_translation[4],next_rotation[4];
  for (unsigned slot=0;slot<4;++slot) {
    if (nodes[slot]>=count || !tl::math::Finite(value.translation[slot]) || value.translation[slot]<0 ||
        !tl::math::Finite(value.rotation[slot]) || value.rotation[slot]<0) return false;
    for (unsigned prior=0;prior<slot;++prior) if (nodes[prior]==nodes[slot]) return false;
    if (!tl::math::Finite(translation[nodes[slot]]) || translation[nodes[slot]]<0 ||
        !tl::math::Finite(rotation[nodes[slot]]) || rotation[nodes[slot]]<0) return false;
    next_translation[slot]=translation[nodes[slot]]+value.translation[slot];
    next_rotation[slot]=rotation[nodes[slot]]+value.rotation[slot];
    if (!tl::math::Finite(next_translation[slot]) || !tl::math::Finite(next_rotation[slot])) return false;
  }
  for (unsigned slot=0;slot<4;++slot) {
    translation[nodes[slot]]=next_translation[slot];
    rotation[nodes[slot]]=next_rotation[slot];
  }
  return true;
}
} // namespace tl::fea::qbat::mapped
