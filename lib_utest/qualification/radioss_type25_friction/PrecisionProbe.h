// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Friction.h"
#ifdef __FAST_MATH__
#error The public TYPE25 usage target must disable fast-math in consumers.
#endif
namespace type25_precision_test {
#if defined(__CUDACC__)
__host__ __device__ __noinline__ inline double ProductSum(double a, double b, double c) {
#else
__attribute__((noinline)) inline double ProductSum(double a, double b, double c) {
#endif
  return a * b + c;
}
}
