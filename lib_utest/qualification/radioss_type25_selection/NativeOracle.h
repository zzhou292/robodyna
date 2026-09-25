// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/selection/Types.h"
#include "lib_src/collision/radioss_type25/selection/ContinuationTypes.h"
#include "lib_src/collision/radioss_type25/selection/NewImpactTypes.h"
namespace type25_selection_test {
namespace n = tlfea::contact::radioss_type25;
namespace s = n::selection;
// Debug-only raw scratch observations. Invalid channels retain the supplied
// fixture seed; that seed is never presented as a native numerical result.
struct RetainedScratchObservation {
  double raw_lb[4]{}, raw_lc[4]{}, clamped_lb[4]{}, clamped_lc[4]{};
  double cache_lb[4]{}, cache_lc[4]{};
};
// Serial qualification-only call: native common observation storage is private
// to the reference library. One occurrence consumes one supplied row snapshot.
s::NativeRetainedResult OracleRetained(const s::Profile&, const s::NativePairInput&,
    const n::NativeGeometryHistory& prior, double scratch_seed = 0.,
    RetainedScratchObservation* observation = nullptr);
// Same serial qualification boundary, complete COR21/DST21/GLOB phase.
s::NativeContinuationResult OracleContinuation(const s::Profile&,
    const s::NativeContinuationInput&, const n::NativeGeometryHistory& prior,
    double scratch_seed = 0., RetainedScratchObservation* observation = nullptr);
struct NewImpactScratchObservation {
  double penetration = 0, lb = 0, lc = 0;
  int far = 0;
};
struct NewImpactOracleStorage {
  std::size_t main_slots = 0, reference_slots = 0;
  std::size_t table_bytes = 0; // C++ plus explicit Fortran unpacked table arrays.
};
s::NativeNewImpactResult OracleNewImpact(const s::Profile&, const s::NativeNewImpactInput&,
    const n::NativeGeometryHistory& prior, double scratch_seed = 0.,
    NewImpactScratchObservation* observation = nullptr,
    NewImpactOracleStorage* storage = nullptr);
} // namespace type25_selection_test
