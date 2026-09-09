#pragma once

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>

// Runtime-only error helper extracted unchanged from cuda_utils.h. Element
// kernels that do not use sparse/direct solvers need no cuDSS/math headers.
#ifndef HANDLE_ERROR_MACRO
#define HANDLE_ERROR_MACRO
static inline void HandleError(cudaError_t err, const char *file, int line) {
  if (err != cudaSuccess) {
    printf("%s in %s at line %d\n", cudaGetErrorString(err), file, line);
    exit(EXIT_FAILURE);
  }
}
#define HANDLE_ERROR(err) (HandleError(err, __FILE__, __LINE__))
#endif
