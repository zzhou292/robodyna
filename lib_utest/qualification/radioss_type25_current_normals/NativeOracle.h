// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/current_normals/Types.h"
#include <array>
#include <vector>
namespace type25_current_normals_test {
namespace n=tlfea::contact::radioss_type25;
namespace c=n::current_normals;
struct NativeResult {
  std::vector<n::StoredNormal> flag1_normals,normals;
  std::vector<n::startup::NormalReference> references;
  std::vector<int> primary_skip; // Actual FLAG1 TAGE, primary prefix only.
  std::vector<std::array<int,4>> free_edges; // Native main/edge and two slot numbers.
  bool finite=true;
};
// Serial qualification only: shares the existing startup oracle COMMON state.
// Explicit prior cache bits are supplied, never manufactured by production.
// Complete original NORMP FLAG1/2 and FREE_BOUND execute in that one oracle.
// Inactive WNOD scratch is deliberately not exposed as defined observations.
NativeResult Oracle(const c::Input&);
// Explicit mixed scalar marshalling only; complete unchanged native NORMP
// receives actualG, encoded roles and supplied cache/activity.
NativeResult OracleMixed(const c::Input&);
}
