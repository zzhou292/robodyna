// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/selection/lifecycle/Types.h"
#include <vector>
namespace type25_lifecycle_test {
namespace n = tlfea::contact::radioss_type25;
namespace s = n::selection;
namespace l = s::lifecycle;
struct OracleResult {
  std::vector<l::RowResult> rows;
  std::vector<l::Occurrence> occurrences;
  // Aligned with occurrences. Only selected.enabled entries are native values;
  // unused slots are API zeros and must not establish numerical observations.
  std::vector<n::NativeRawGeometryResult> geometry;
};
// Serial qualification-only composition of independent native references.
// Bounds: 32 main tables (inherited BeginOracle), 128 rows, 256 nodes/references,
// 4096 initial occurrences and 8192 entries in each source CSR.
// A source-undefined selected geometry throws from its existing native oracle;
// the witness remains a composed admission failure, never a fabricated point.
OracleResult OracleLifecycle(const l::Input&);
// Explicit post-response/assembly source phase. Never called by OracleLifecycle.
std::vector<l::RowResult> OracleFinish(const std::vector<l::RowResult>&);
} // namespace type25_lifecycle_test
