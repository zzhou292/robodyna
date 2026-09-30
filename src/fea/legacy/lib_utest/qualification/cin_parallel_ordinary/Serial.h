// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_advance/Input.h"
namespace tl::fea::cin_parallel_test {
void RunSerialHost(const cin_advance::Input&);
cudaError_t LaunchSerial(const cin_advance::Input&, cudaStream_t);
}
