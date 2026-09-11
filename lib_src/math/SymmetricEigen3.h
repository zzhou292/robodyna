// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "Fixed3.h"
#include "Quaternion.h"
#include <Eigen/Eigenvalues>

namespace tl::math {
// Reuse Eigen's fixed-size solver, also used by Chrono and TL rigid aggregates.
// Columns are eigenvectors; eigenvalues are ascending. No dynamic allocation.
struct SymmetricSpectrum3 {
  double value[3]{};
  Matrix3 vectors{};
};
#if defined(__CUDACC__)
__host__ __device__
#endif
inline bool SymmetricEigen3(const double (&tensor)[6], SymmetricSpectrum3& output) noexcept {
  for (double x : tensor) if (!Finite(x)) return false;
  Eigen::Matrix<double,3,3> matrix;
  matrix << tensor[0],tensor[3],tensor[5],
            tensor[3],tensor[1],tensor[4],
            tensor[5],tensor[4],tensor[2];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,3,3>> solver;
  // Eigen 3.4's iterative 3x3 tridiagonal specialization is host-only.
  // Its fixed 3x3 direct solver has the required host/device implementation.
  solver.computeDirect(matrix, Eigen::ComputeEigenvectors);
  if (solver.info() != Eigen::Success) return false;
  SymmetricSpectrum3 next;
  for (unsigned k=0;k<3;++k) {
    next.value[k]=solver.eigenvalues()[k];
    if (!Finite(next.value[k])) return false;
    for (unsigned r=0;r<3;++r) {
      next.vectors.v[3*r+k]=solver.eigenvectors()(r,k);
      if (!Finite(next.vectors.v[3*r+k])) return false;
    }
  }
  output=next;
  return true;
}
} // namespace tl::math
