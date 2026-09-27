// SPDX-License-Identifier: AGPL-3.0-or-later
// Compile the actual owning body. The test-only suffix exposes its final fold.
#include "lib_src/elements/solids/resident/Candidate.cu"
namespace tl::fea::solids::batch_detail {
void LaunchFinalizeTest(Storage* state, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity) {
  Finalize<<<1, measurement::Threads, 0, view.stream>>>(state, accepted, trial, view, identity, false);
}
}
