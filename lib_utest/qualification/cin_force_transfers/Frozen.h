#pragma once
#include "lib_src/solvers/cin_advance/Input.h"
namespace tl::fea::cin_transfer_test {
cudaError_t LaunchFrozen(const cin_advance::Input&, cudaStream_t);
}
