// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidKineticValues.h"
namespace tl::fea::rigid {
// Supplied phase and values only: no clock, source authentication, eigen solve,
// state reconstruction, allocation or publication. On failure output is intact.
// Stored-midpoint energy explicitly pairs midpoint velocities with the supplied
// lagged force-stage axes; it is not the native collocated RGBCOR diagnostic.
TL_RIGID_OBSERVATION_HD inline ObservationReport ObserveGroupKinetic(
    const GroupKineticInput& input,GroupKineticObservation& output) {
  if(!observation_detail::ValidPhase(input.phase)) return {ObservationStatus::UnsupportedPhase};
  return observation_detail::KineticValues(input,output);
}
} // namespace tl::fea::rigid
