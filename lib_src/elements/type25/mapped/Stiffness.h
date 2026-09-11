// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected R2LEN3/R4CUM3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "../Type25Property.h"
#if defined(__CUDACC__)
#define TL_TYPE25_MAPPED_HD __host__ __device__
#else
#define TL_TYPE25_MAPPED_HD
#endif

namespace tl::fea::type25::mapped {
struct NodalStiffness {
  double translation=0;
  double rotation=0;
};
// R6DEF3's pre-floor coefficients are already retained in each Evaluation,
// including the separately prepared virgin coefficient packet. For C=0 the
// R2LEN3 nodal branch returns K at either endpoint independently of MS/IN.
// It uses NEW OFF after failure, while the current interval force may remain.
TL_TYPE25_MAPPED_HD inline bool AcceptedStiffness(const Property& property,
    const Evaluation& accepted,NodalStiffness& output) noexcept {
  for (double damping:property.damping) if (damping!=0) return false;
  if (!detail::Positive(accepted.translation_stiffness_N_per_m) ||
      !detail::Positive(accepted.rotation_stiffness_Nm_per_rad)) return false;
  NodalStiffness next;
  if (accepted.history.active) {
    next.translation=accepted.translation_stiffness_N_per_m;
    next.rotation=accepted.rotation_stiffness_Nm_per_rad;
  }
  output=next;
  return true;
}
// Two distinct ordered endpoint slots. Validate both additions before writes;
// shared nodes across connections retain original connection/endpoint order.
TL_TYPE25_MAPPED_HD inline bool AddStiffness(const std::size_t (&nodes)[2],
    const NodalStiffness& value,double* translation,double* rotation,std::size_t count) noexcept {
  if (!translation || !rotation || translation==rotation || nodes[0]==nodes[1] ||
      !detail::Nonnegative(value.translation) || !detail::Nonnegative(value.rotation)) return false;
  double next_translation[2],next_rotation[2];
  for (unsigned endpoint=0;endpoint<2;++endpoint) {
    const auto node=nodes[endpoint];
    if (node>=count || !detail::Nonnegative(translation[node]) ||
        !detail::Nonnegative(rotation[node])) return false;
    next_translation[endpoint]=translation[node]+value.translation;
    next_rotation[endpoint]=rotation[node]+value.rotation;
    if (!detail::Nonnegative(next_translation[endpoint]) ||
        !detail::Nonnegative(next_rotation[endpoint])) return false;
  }
  for (unsigned endpoint=0;endpoint<2;++endpoint) {
    translation[nodes[endpoint]]=next_translation[endpoint];
    rotation[nodes[endpoint]]=next_rotation[endpoint];
  }
  return true;
}
} // namespace tl::fea::type25::mapped
#undef TL_TYPE25_MAPPED_HD
