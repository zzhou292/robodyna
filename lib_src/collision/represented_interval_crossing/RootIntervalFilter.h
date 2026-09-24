// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossingTypes.h"

namespace tlfea::contact::represented_interval_crossing {
// Private value helper. The native owner supplies canonical facet/vertex order;
// this is neither a caller-owned certificate nor physical publication authority.
struct RootIntervalInput { Vec3 vertices[2][2][3]{}; }; // facet, endpoint, vertex
struct RootIntervalResult {
  bool separated = false;
  Vec3 axis{}; // represented fixed direction, for independent qualification
};
// Binary64 RN, no reassociation/FMA/FTZ. Every arithmetic ambiguity falls back.
RootIntervalResult ProveRootIntervalSeparation(const RootIntervalInput&) noexcept;
} // namespace tlfea::contact::represented_interval_crossing
