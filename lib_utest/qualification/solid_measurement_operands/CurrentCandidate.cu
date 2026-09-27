// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/solid_candidate_validation/CurrentCandidate.cu"
namespace tl::fea::solids::batch_detail {
void LaunchOperandFinalizeTest(Storage* state, unsigned accepted, unsigned trial,
    NodalPreparedView view, BatchDiagnostics identity) {
  Finalize<<<1, measurement::Threads, 0, view.stream>>>(state, accepted, trial, view, identity, false, true);
}
}
