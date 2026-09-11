// SPDX-License-Identifier: AGPL-3.0-or-later
// SETMATRANS/CKKTRAN3V/BMTRL2G/S8EL2GNJ, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "ReferenceTypes.h"
#include "lib_src/elements/solid18/Solid18ForceTypes.h"

namespace tl::fea::solid18::total_strain::detail {
struct ReferenceTransform {
  double qt[3][3]{};
  double qc[3][3]{};
  double qgc[3][3]{};
};

TL_SOLID18_HD inline ReferenceTransform MakeTransform(const Matrix3& frame) noexcept {
  ReferenceTransform result;
  for (unsigned j = 0; j < 3; ++j) {
    const double e1 = frame.v[3*j];
    const double e2 = frame.v[3*j+1];
    const double e3 = frame.v[3*j+2];
    result.qt[0][j] = e1;
    result.qt[1][j] = e2;
    result.qt[2][j] = e3;
    result.qc[0][j] = e1*e1;
    result.qc[1][j] = e2*e2;
    result.qc[2][j] = e3*e3;
    result.qgc[0][j] = e1*e2;
    result.qgc[1][j] = e2*e3;
    result.qgc[2][j] = e1*e3;
  }
  return result;
}

TL_SOLID18_HD inline void TransformMatrix(const double (&left)[3][3],
    const double (&right)[3][3], double (&matrix)[3][3]) noexcept {
  double next[3][3];
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned j = 0; j < 3; ++j) {
      next[i][j] = left[0][i]*(matrix[0][0]*right[0][j]+
          matrix[0][1]*right[1][j]+matrix[0][2]*right[2][j])+
          left[1][i]*(matrix[1][0]*right[0][j]+
          matrix[1][1]*right[1][j]+matrix[1][2]*right[2][j])+
          left[2][i]*(matrix[2][0]*right[0][j]+
          matrix[2][1]*right[1][j]+matrix[2][2]*right[2][j]);
    }
  }
  for (unsigned i = 0; i < 3; ++i) {
    for (unsigned j = 0; j < 3; ++j) matrix[i][j] = next[i][j];
  }
}

TL_SOLID18_HD inline Status TransformPoint(const ReferenceTransform& q,
    const PointDerivatives& point, double (&pij)[72]) noexcept {
  for (unsigned n = 0; n < 8; ++n) {
    const auto& p = point.regular_per_m;
    const auto& b = point.cross_per_m;
    double bm[3][3] = {{p[0][n], b[1][n], b[3][n]},
                       {b[0][n], p[1][n], b[5][n]},
                       {b[2][n], b[4][n], p[2][n]}};
    // The complete native body fills BC with regular PX/PY/PZ, despite
    // receiving selective PXY/PYX/etc arguments. Preserve those assignments.
    double bc[3][3] = {{p[1][n], p[0][n], 0},
                       {0, p[2][n], p[1][n]},
                       {p[2][n], 0, p[0][n]}};
    TransformMatrix(q.qc, q.qt, bm);
    TransformMatrix(q.qgc, q.qt, bc);
    for (unsigned i = 0; i < 3; ++i) {
      for (unsigned j = 0; j < 3; ++j) {
        bm[i][j] = bm[i][j]+bc[i][j];
        if (!tl::math::Finite(bm[i][j])) return Status::NonfiniteResult;
      }
    }
    pij[3*n] = bm[0][0];
    pij[3*n+1] = bm[1][1];
    pij[3*n+2] = bm[2][2];
    pij[24+6*n] = bm[1][0];
    pij[25+6*n] = bm[0][1];
    pij[26+6*n] = bm[2][0];
    pij[27+6*n] = bm[0][2];
    pij[28+6*n] = bm[2][1];
    pij[29+6*n] = bm[1][2];
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::total_strain::detail
