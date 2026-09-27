// SPDX-License-Identifier: AGPL-3.0-or-later
// Actual production finalizer; the test suffix changes no kernel body.
#include "lib_src/elements/solids/resident/Candidate.cu"
namespace tl::fea::solids::batch_detail {
void LaunchControlFinalizeTest(Storage* state,unsigned accepted,unsigned trial,
    NodalPreparedView view,BatchDiagnostics identity,bool initial,bool operands) {
  Finalize<<<1,measurement::Threads,0,view.stream>>>(state,accepted,trial,view,identity,initial,operands);
}
}
