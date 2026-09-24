// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/math/HostDevice.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <tuple>
#if defined(__CUDACC__)
#include <cuda/std/algorithm>
#include <cuda/std/cstring>
#include <cuda/std/limits>
#include <cuda/std/tuple>
#endif

namespace tlfea::contact::represented_interval_crossing {
// Reuse the shipped standard-library equivalents, including exact lexicographic
// source-key ordering. Both ordinary C++ and nvcc host passes retain std, so
// host inline definitions have identical lookup across translation units.
#if defined(__CUDA_ARCH__)
namespace portable = ::cuda::std;
#else
namespace portable = ::std;
#endif
}  // namespace tlfea::contact::represented_interval_crossing
