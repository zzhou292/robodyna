// =============================================================================
// Rotation derivatives adapted from Project Chrono's ChRotUtils.h.
// Copyright (c) 2014 projectchrono.org. All rights reserved.
// Authors of the source: Alessandro Tasora, Radu Serban; adapted from MBDyn.
// Source: chrono/src/chrono/fea/ChRotUtils.h, SHA256
// 103c30ae584ba3a36bb8ca442ec3feb3079a1cba5cb41dece9ae8c3abd047875.
// Matrix cross products follow chrono/core/ChMatrixMBD.h. This adaptation uses
// the existing TL fixed-size frame values, binary64 host/device arithmetic,
// explicit input admission and staged publication. The source coefficient
// tables, summation order and operation-specific series thresholds are kept.
// Only force-path operations through coefficient E are carried, not tangents.
//
// Copyright (c) 2016, Project Chrono Development Team
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//  - Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  - Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  - Neither the name of the nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// =============================================================================

#pragma once

#include "ReissnerFrame.h"

#if defined(__CUDACC__)
#define TL_REISSNER_ROTATION_HD __host__ __device__
#else
#define TL_REISSNER_ROTATION_HD
#endif

namespace tl::fea::reissner {
namespace detail {

// Small operations shared with the Q4 force path; no owning math objects.
TL_REISSNER_ROTATION_HD inline bool Finite(Vec3 a) {
  return Finite(a.x) && Finite(a.y) && Finite(a.z);
}
TL_REISSNER_ROTATION_HD inline double Component(Vec3 a, unsigned axis) {
  return axis == 0 ? a.x : (axis == 1 ? a.y : a.z);
}
TL_REISSNER_ROTATION_HD inline Vec3 Add(Vec3 a, Vec3 b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
TL_REISSNER_ROTATION_HD inline Vec3 Subtract(Vec3 a, Vec3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
TL_REISSNER_ROTATION_HD inline Vec3 Scale(Vec3 a, double scale) {
  return {a.x * scale, a.y * scale, a.z * scale};
}
TL_REISSNER_ROTATION_HD inline double Dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
TL_REISSNER_ROTATION_HD inline Vec3 Cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
TL_REISSNER_ROTATION_HD inline Matrix3 Identity() {
  return {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
}
TL_REISSNER_ROTATION_HD inline Matrix3 Add(const Matrix3& a, const Matrix3& b) {
  Matrix3 result;
  for (unsigned i = 0; i < 9; ++i) result.v[i] = a.v[i] + b.v[i];
  return result;
}
TL_REISSNER_ROTATION_HD inline Matrix3 Subtract(const Matrix3& a, const Matrix3& b) {
  Matrix3 result;
  for (unsigned i = 0; i < 9; ++i) result.v[i] = a.v[i] - b.v[i];
  return result;
}
TL_REISSNER_ROTATION_HD inline Matrix3 Scale(const Matrix3& a, double scale) {
  Matrix3 result;
  for (unsigned i = 0; i < 9; ++i) result.v[i] = a.v[i] * scale;
  return result;
}
TL_REISSNER_ROTATION_HD inline Matrix3 Transpose(const Matrix3& a) {
  return {{a.v[0], a.v[3], a.v[6], a.v[1], a.v[4], a.v[7], a.v[2], a.v[5], a.v[8]}};
}
TL_REISSNER_ROTATION_HD inline Matrix3 Product(const Matrix3& a, const Matrix3& b) {
  Matrix3 result;
  for (unsigned row = 0; row < 3; ++row)
    for (unsigned col = 0; col < 3; ++col)
      result.v[3 * row + col] = a.v[3 * row] * b.v[col] +
                               a.v[3 * row + 1] * b.v[3 + col] +
                               a.v[3 * row + 2] * b.v[6 + col];
  return result;
}
TL_REISSNER_ROTATION_HD inline Vec3 Product(const Matrix3& a, Vec3 b) {
  return {a.v[0] * b.x + a.v[1] * b.y + a.v[2] * b.z,
          a.v[3] * b.x + a.v[4] * b.y + a.v[5] * b.z,
          a.v[6] * b.x + a.v[7] * b.y + a.v[8] * b.z};
}
TL_REISSNER_ROTATION_HD inline Matrix3 Skew(Vec3 a) {
  return {{0, -a.z, a.y, a.z, 0, -a.x, -a.y, a.x, 0}};
}
TL_REISSNER_ROTATION_HD inline Matrix3 Outer(Vec3 a, Vec3 b) {
  return {{a.x * b.x, a.x * b.y, a.x * b.z,
           a.y * b.x, a.y * b.y, a.y * b.z,
           a.z * b.x, a.z * b.y, a.z * b.z}};
}
// [a]_x [b]_x, with the donor's two-term diagonal summation order.
TL_REISSNER_ROTATION_HD inline Matrix3 DoubleSkew(Vec3 a, Vec3 b) {
  return {{-a.y * b.y - a.z * b.z, a.y * b.x, a.z * b.x,
           a.x * b.y, -a.z * b.z - a.x * b.x, a.z * b.y,
           a.x * b.z, a.y * b.z, -a.x * b.x - a.y * b.y}};
}

TL_REISSNER_ROTATION_HD inline Status AdmitRotationVector(Vec3 phi) {
  if (!Finite(phi)) return Status::kNonfiniteInput;
  constexpr double pi = 3.14159265358979323846;
  // The strict principal chart excludes the log branch cut and inverse-J
  // singularities. The element's physical director chart remains stricter.
  if (!(Dot(phi, phi) < pi * pi)) return Status::kOutsideChart;
  return Status::kSuccess;
}

// Internal caller supplies 1 <= count <= 5 and an admitted phi. Chrono chooses
// ONE branch using the highest requested coefficient, including lower terms.
TL_REISSNER_ROTATION_HD inline void RotationCoefficients(unsigned count, Vec3 phi, double* coefficients) {
  const double denominators[5][9] = {
      {1., -6., 120., -5040., 362880., -39916800., 6227020800., -1307674368000., 355687428096000.},
      {2., -24., 720., -40320., 3628800., -479001600., 87178291200., -20922789888000., 6402373705728000.},
      {6., -120., 5040., -362880., 39916800., -6227020800., 1307674368000., -355687428096000., 121645100408832000.},
      {-12., 180., -6720., 453600., -47900160., 7264857600., -1494484992000., 400148356608000., -135161222676480000.},
      {-60., 1260., -60480., 4989600., -622702080., 108972864000., -25406244864000., 7602818775552000., -2838385676206080000.}};
  const double thresholds[5] = {1.1, 1.3, 1.5, 1.6, 1.7};
  const double square = Dot(phi, phi);
  const double angle = ::sqrt(square);
  if (angle < thresholds[count - 1]) {
    double powers[9];
    powers[0] = 1;
    for (unsigned j = 1; j < 9; ++j) powers[j] = powers[j - 1] * square;
    for (unsigned k = 0; k < count; ++k) {
      coefficients[k] = 0;
      for (unsigned j = 0; j < 9; ++j) coefficients[k] += powers[j] / denominators[k][j];
    }
    return;
  }
  coefficients[0] = ::sin(angle) / angle;
  if (count == 1) return;
  coefficients[1] = (1 - ::cos(angle)) / square;
  if (count == 2) return;
  coefficients[2] = (1 - coefficients[0]) / square;
  if (count == 3) return;
  coefficients[3] = (coefficients[0] - coefficients[1] * 2) / square;
  if (count == 4) return;
  coefficients[4] = (coefficients[1] - coefficients[2] * 3) / square;
}

}  // namespace detail

// Checked binary64 host/device operations. No allocation, state or history.
// Every failure leaves caller outputs unchanged. Paired outputs must be
// distinct objects. No fast-math/finite-only arithmetic is admitted. Vec3 phi
// represents a principal rotation vector with |phi| < pi; no normalization or
// chart repair is performed. Rotation-vector increments and matrix spins are
// expressed in the same frame, with delta(R) R^T = [J(phi) delta(phi)]_x.
TL_REISSNER_ROTATION_HD inline Status ComputeRotationAndJacobian(Vec3 phi, Matrix3& rotation, Matrix3& jacobian) {
  const auto status = detail::AdmitRotationVector(phi);
  if (status != Status::kSuccess) return status;
  double coefficients[3];
  detail::RotationCoefficients(3, phi, coefficients);
  const auto candidate_rotation = detail::Add(
      detail::Add(detail::Identity(), detail::Skew(detail::Scale(phi, coefficients[0]))),
      detail::DoubleSkew(phi, detail::Scale(phi, coefficients[1])));
  const auto candidate_jacobian = detail::Add(
      detail::Add(detail::Identity(), detail::Skew(detail::Scale(phi, coefficients[1]))),
      detail::DoubleSkew(phi, detail::Scale(phi, coefficients[2])));
  if (!detail::Finite(candidate_rotation) || !detail::Finite(candidate_jacobian)) return Status::kNonfiniteResult;
  rotation = candidate_rotation;
  jacobian = candidate_jacobian;
  return Status::kSuccess;
}

TL_REISSNER_ROTATION_HD inline Status ComputeRotationJacobianInverse(Vec3 phi, Matrix3& inverse) {
  const auto status = detail::AdmitRotationVector(phi);
  if (status != Status::kSuccess) return status;
  double coefficients[4];
  detail::RotationCoefficients(4, phi, coefficients);
  const double c_star = -coefficients[3] / (coefficients[1] * 2);
  const auto candidate = detail::Add(
      detail::Add(detail::Identity(), detail::Skew(detail::Scale(phi, -.5))),
      detail::DoubleSkew(phi, detail::Scale(phi, c_star)));
  if (!detail::Finite(candidate)) return Status::kNonfiniteResult;
  inverse = candidate;
  return Status::kSuccess;
}

// Principal log of a proper rotation. Rejects nonorthogonal/reflected matrices
// (max entry residual in R^T R and |det(R)-1| must each be <= 1e-10) and the
// exact pi branch cut. Arbitrary matrices are never projected onto SO(3).
TL_REISSNER_ROTATION_HD inline Status ComputeRotationVector(const Matrix3& rotation, Vec3& phi) {
  if (!detail::Finite(rotation)) return Status::kNonfiniteInput;
  const auto gram = detail::Product(detail::Transpose(rotation), rotation);
  for (unsigned row = 0; row < 3; ++row)
    for (unsigned col = 0; col < 3; ++col)
      if (!(::fabs(gram.v[3 * row + col] - (row == col ? 1.0 : 0.0)) <= 1e-10))
        return Status::kOutsideChart;
  const auto& r = rotation.v;
  const double determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) -
                             r[1] * (r[3] * r[8] - r[5] * r[6]) +
                             r[2] * (r[3] * r[7] - r[4] * r[6]);
  if (!(::fabs(determinant - 1) <= 1e-10)) return Status::kOutsideChart;
  const double cosine = (r[0] + r[4] + r[8] - 1) / 2;
  Vec3 candidate;
  if (cosine > 0) {
    const Vec3 unit{.5 * (r[7] - r[5]), .5 * (r[2] - r[6]), .5 * (r[3] - r[1])};
    const double angle = ::atan2(::sqrt(detail::Dot(unit, unit)), cosine);
    double a;
    detail::RotationCoefficients(1, Vec3{angle, 0, 0}, &a);
    candidate = detail::Scale(unit, 1 / a);
  } else {
    auto eet = detail::Scale(detail::Add(rotation, detail::Transpose(rotation)), .5);
    eet.v[0] -= cosine;
    eet.v[4] -= cosine;
    eet.v[8] -= cosine;
    unsigned column = 0;
    if (eet.v[4] > eet.v[0]) column = 1;
    if (eet.v[8] > eet.v[4 * column]) column = 2;
    const auto unit = detail::Scale(Vec3{eet.v[column], eet.v[3 + column], eet.v[6 + column]},
                                    1 / ::sqrt(eet.v[4 * column] * (1 - cosine)));
    const auto product = detail::Product(detail::Skew(unit), rotation);
    const double sine = -(product.v[0] + product.v[4] + product.v[8]) / 2;
    const double angle = ::atan2(sine, cosine);
    constexpr double pi = 3.14159265358979323846;
    if (!(::fabs(angle) < pi)) return Status::kOutsideChart;
    candidate = detail::Scale(unit, angle);
  }
  if (!detail::Finite(candidate)) return Status::kNonfiniteResult;
  const auto status = detail::AdmitRotationVector(candidate);
  if (status != Status::kSuccess) return status;
  phi = candidate;
  return Status::kSuccess;
}

// Chrono Elle(phi,a): delta(J(phi)) a = L(phi,a) delta(phi), with a held fixed.
// This is the matrix of derivatives of J(phi)*a, not delta(J) in direction a.
TL_REISSNER_ROTATION_HD inline Status ComputeRotationJacobianVariation(Vec3 phi, Vec3 a, Matrix3& variation) {
  const auto status = detail::AdmitRotationVector(phi);
  if (status != Status::kSuccess) return status;
  if (!detail::Finite(a)) return Status::kNonfiniteInput;
  double coefficients[5];
  detail::RotationCoefficients(5, phi, coefficients);
  auto candidate = detail::Skew(detail::Scale(a, -coefficients[1]));
  candidate = detail::Subtract(candidate, detail::Add(
      detail::DoubleSkew(phi, detail::Scale(a, coefficients[2])),
      detail::Skew(detail::Cross(phi, detail::Scale(a, coefficients[2])))));
  candidate = detail::Add(candidate, detail::Add(
      detail::Outer(detail::Cross(phi, a), detail::Scale(phi, coefficients[3])),
      detail::Outer(detail::Product(detail::DoubleSkew(phi, phi), a), detail::Scale(phi, coefficients[4]))));
  if (!detail::Finite(candidate)) return Status::kNonfiniteResult;
  variation = candidate;
  return Status::kSuccess;
}

}  // namespace tl::fea::reissner

#undef TL_REISSNER_ROTATION_HD
