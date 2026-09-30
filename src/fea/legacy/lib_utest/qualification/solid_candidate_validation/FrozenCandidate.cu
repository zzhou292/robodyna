// SPDX-License-Identifier: AGPL-3.0-or-later
#include <FrozenCandidate.cu>
namespace tl::fea::solids::batch_detail::frozen {
void LaunchFinalizeTest(Storage* state, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity) {
  Finalize<<<1, 1, 0, view.stream>>>(state, accepted, trial, view, identity, false);
}
}
