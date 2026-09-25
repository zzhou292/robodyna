// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Packet.h"
namespace qeph_projection_test {
// Independent original native routines, under the existing shared qualification
// context mutex. Selected explicit controls are checked before the C boundary.
// These wrappers do not call production geometry/projection implementations.
RateResult NativeRates(const RateInput&);
ForceResult NativeForces(const ForceInput&);
} // namespace qeph_projection_test
