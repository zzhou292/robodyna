// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::startup::role_policy {
TL_MATH_HOST_DEVICE inline bool Valid(ShellSideRole role) {
  return role == ShellSideRole::Ordinary || role == ShellSideRole::CoatingForward ||
      role == ShellSideRole::CoatingReversed;
}
TL_MATH_HOST_DEVICE inline bool Resolved(TopologyPolicy policy) {
  return policy == TopologyPolicy::NativeResolvedShellSides;
}
TL_MATH_HOST_DEVICE inline bool Supported(Profile profile, TopologyPolicy policy) {
  if (profile == Profile::ResolvedShellSides) return Resolved(policy);
  return (profile == Profile::OrdinaryExteriorFixedMain || profile == Profile::OrdinaryExteriorMovingMain) &&
      (policy == TopologyPolicy::ManifoldTwoSided || policy == TopologyPolicy::NativeOrdinaryShell);
}
} // namespace tlfea::contact::radioss_type25::startup::role_policy
