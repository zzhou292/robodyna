// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/startup/Types.h"
#include <array>
#include <vector>
namespace type25_startup_test {
namespace n=tlfea::contact::radioss_type25;
namespace s=n::startup;
struct NativeResult {
  std::vector<s::Main> mains;
  std::vector<std::uint32_t> expanded_to_primary,primary_to_partner,offsets,incidence;
  std::vector<n::StoredNormal> starter_normals,ready_normals;
  std::vector<s::NormalReference> starter_references,ready_references;
  std::vector<s::ShellSideRole> primary_roles; // Explicit input provenance for resolved scope only.
  std::array<float,4> floors{};
  int warning_count=0,selector_calls=0;
  std::array<int,2> warning_node_ids{};
};
// Serial qualification-only original Fortran composition, bounded to256 nodes
// and160 primaries. External node IDs must fit the original native integer ABI.
// Caller supplies valid admitted ordinary source topology; no production
// topology/normal helper supplies these expected values.
NativeResult Oracle(const s::Input&,const double* expanded_coefficients,std::size_t count);
} // namespace type25_startup_test
