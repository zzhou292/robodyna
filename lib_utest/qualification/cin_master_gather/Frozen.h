// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_advance/Input.h"
namespace tl::fea::cin_gather_test {
cudaError_t LaunchFrozen(const cin_advance::Input&,cudaStream_t);
}
