#pragma once
#include "lib_src/solvers/FENodalStateStorage.h"
namespace tl::fea::reset_test {
void FrozenLaunch(nodal_detail::Control*, std::uint64_t, std::uint64_t, cudaStream_t);
}
