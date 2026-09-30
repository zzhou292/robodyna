// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// Same annotation rule as the existing math/Fixed3Operations.h helpers.
// No CUDA runtime include and no different host/device type definition.
#if defined(__CUDACC__)
#define TL_MATH_HOST_DEVICE __host__ __device__
#else
#define TL_MATH_HOST_DEVICE
#endif
