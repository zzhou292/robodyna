// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_parallel_ordinary/Serial.h"
#include "lib_src/constraints/tied_shell/runtime/CinForceStage.h"
#include "serial/Force.h"

namespace tl::constraints::tied_shell::cin {
// Test-only bridge substitutes only the complete frozen force stage call in
// the existing complete frozen caller. No production helper is its oracle.
TL_TIED_PATCH_HD inline StageReport PrepareFrozenForceTrial(StageView model, ForceTrial trial) noexcept {
  return cin_input_frozen::PrepareForceTrial(model, trial);
}
}
namespace tl::fea::cin_input_test {
void RunFrozenHost(const cin_advance::Input&);
cudaError_t LaunchFrozen(const cin_advance::Input&, cudaStream_t);
}
