// =============================================================================
// Adapted from Project Chrono's locally qualified ChReissnerFrame.h/.cpp.
// Copyright (c) 2026 projectchrono.org. All rights reserved.
// Source paths: chrono/src/chrono/fea/ChReissnerFrame.{h,cpp}.
// Source SHA256 (header):
// 90e6769b67cdb6f287d0f38cdea0c74e631600cb2d951765608160d5a805323b
// Source SHA256 (implementation):
// 360f58c96fa24cb46dc044592adee848e777cc23a8c53d7ade859697b0834a30
// These identify the local qualified helper, not an upstream release.
// Quaternion product/world derivative and rotation-matrix arithmetic follow
// Chrono core ChQuaternion.h/.cpp and ChMatrix33.h. Changes here replace
// Eigen/Chrono objects with fixed-size binary64 values and host/device inline
// functions; the chart, first variations and failure order are retained.
// No element force, tangent, material history, mass or dynamics is implemented.
//
// The Chrono distribution license is reproduced for this adaptation:
// Copyright (c) 2016, Project Chrono Development Team
// All rights reserved.
//
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
//
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

#include "lib_src/math/Quaternion.h"

#include <cfloat>
#include <cmath>

#if defined(__CUDACC__)
#define TL_REISSNER_HD __host__ __device__
#else
#define TL_REISSNER_HD
#endif

namespace tl::fea::reissner {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
using Quaternion = tl::math::Quaternion;
struct Matrix3 {
  double v[9]{};  // Row-major; default is the zero matrix.
};
struct SpinJacobian {
  Matrix3 node[4]{};
};
struct MeanFrame {
  Quaternion rotation;
  Matrix3 frame{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  // WORLD delta(theta_mean) = sum_n spin.node[n] delta(theta_n).
  SpinJacobian spin;
};
enum class Status {
  kSuccess,
  kNonfiniteInput,
  kNonUnitQuaternion,
  kOutsideChart,
  kNonfiniteResult
};

namespace detail {

using tl::math::Finite;
TL_REISSNER_HD inline bool Finite(const Matrix3& matrix) {
  for (unsigned i = 0; i < 9; ++i)
    if (!Finite(matrix.v[i])) return false;
  return true;
}
TL_REISSNER_HD inline bool Finite(const SpinJacobian& matrices) {
  for (unsigned n = 0; n < 4; ++n)
    if (!Finite(matrices.node[n])) return false;
  return true;
}
using tl::math::Add;
using tl::math::Subtract;
using tl::math::Scale;
using tl::math::Dot;
using tl::math::Product;
TL_REISSNER_HD inline Matrix3 Rotation(Quaternion q) {
  const double ww = q.w * q.w, xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  return {{(ww + xx) * 2 - 1, (xy - wz) * 2, (xz + wy) * 2,
           (xy + wz) * 2, (ww + yy) * 2 - 1, (yz - wx) * 2,
           (xz - wy) * 2, (yz + wx) * 2, (ww + zz) * 2 - 1}};
}

TL_REISSNER_HD inline Status CompleteVariation(const Matrix3& common_spin,
                                               const SpinJacobian& frozen,
                                               const SpinJacobian& mean_spin,
                                               SpinJacobian& output) {
  if (!Finite(common_spin) || !Finite(frozen) || !Finite(mean_spin))
    return Status::kNonfiniteInput;
  Matrix3 closure = common_spin;
  for (unsigned n = 0; n < 4; ++n)
    for (unsigned i = 0; i < 9; ++i)
      closure.v[i] -= frozen.node[n].v[i];
  if (!Finite(closure)) return Status::kNonfiniteResult;
  SpinJacobian candidate;
  for (unsigned n = 0; n < 4; ++n)
    for (unsigned row = 0; row < 3; ++row)
      for (unsigned col = 0; col < 3; ++col) {
        const double product = closure.v[3 * row] * mean_spin.node[n].v[col] +
                               closure.v[3 * row + 1] * mean_spin.node[n].v[3 + col] +
                               closure.v[3 * row + 2] * mean_spin.node[n].v[6 + col];
        candidate.node[n].v[3 * row + col] = frozen.node[n].v[3 * row + col] + product;
      }
  if (!Finite(candidate)) return Status::kNonfiniteResult;
  output = candidate;
  return Status::kSuccess;
}

}  // namespace detail

// Checked allocation-free operations in one execution memory space. Every
// spin is WORLD: delta(R) = [delta(theta)]_x R. Failure leaves output unchanged;
// corrections permit output to alias either input. The mean copies its four
// input quaternions before publishing output. The caller supplies four valid
// input objects, owns their lifetime and serializes concurrent writes.
// Use binary64 arithmetic without fast-math/finite-only assumptions. Floating
// results are compared with declared tolerances, not CPU/GPU bit equality.
//
// Equal-weight, sign-aligned normalized quaternion mean. Each q must be finite
// and satisfy |q.q - 1| <= 1e-12. After alignment to node 0, EVERY pair must have
// dot > cos(pi/4), admitting relative director rotations strictly below 90 deg.
// Arbitrary common rigid rotation is allowed. No invalid quaternion is repaired
// and no wider chart is admitted; the output quaternion is sign-equivalent,
// not component-sign canonical.
TL_REISSNER_HD inline Status ComputeMeanFrame(const Quaternion rotation[4], MeanFrame& output) {
  constexpr double unit_tolerance = 1e-12;
  constexpr double minimum_pair_dot = 0.70710678118654752440;
  Quaternion aligned[4] = {rotation[0], rotation[1], rotation[2], rotation[3]};
  for (unsigned n = 0; n < 4; ++n) {
    auto& q = aligned[n];
    if (!detail::Finite(q)) return Status::kNonfiniteInput;
    const double norm_squared = detail::Dot(q, q);
    if (!detail::Finite(norm_squared) || ::fabs(norm_squared - 1) > unit_tolerance)
      return Status::kNonUnitQuaternion;
    if (detail::Dot(q, rotation[0]) < 0) q = detail::Scale(q, -1);
  }
  for (unsigned n = 0; n < 4; ++n)
    for (unsigned m = 0; m < n; ++m)
      if (!(detail::Dot(aligned[n], aligned[m]) > minimum_pair_dot))
        return Status::kOutsideChart;

  Quaternion sum{0, 0, 0, 0};
  for (unsigned n = 0; n < 4; ++n)
    sum = detail::Add(sum, detail::Scale(aligned[n], .25));
  const double length = ::sqrt(detail::Dot(sum, sum));
  if (!detail::Finite(length) || length <= .5) return Status::kNonfiniteResult;
  MeanFrame candidate;
  candidate.rotation = detail::Scale(sum, 1 / length);
  candidate.frame = detail::Rotation(candidate.rotation);
  const Quaternion conjugate{candidate.rotation.w, -candidate.rotation.x,
                             -candidate.rotation.y, -candidate.rotation.z};
  for (unsigned n = 0; n < 4; ++n) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      const Quaternion direction{0, axis == 0 ? 1.0 : 0.0,
                                 axis == 1 ? 1.0 : 0.0, axis == 2 ? 1.0 : 0.0};
      // Chrono QuatDtFromAngVelAbs(direction,q) = .5 * (0,direction) * q.
      const auto d_sum = detail::Scale(detail::Scale(detail::Product(direction, aligned[n]), .5), .25);
      const auto d_mean = detail::Scale(
          detail::Subtract(d_sum, detail::Scale(candidate.rotation, detail::Dot(candidate.rotation, d_sum))), 1 / length);
      const auto spin = detail::Scale(detail::Product(d_mean, conjugate), 2);
      candidate.spin.node[n].v[axis] = spin.x;
      candidate.spin.node[n].v[3 + axis] = spin.y;
      candidate.spin.node[n].v[6 + axis] = spin.z;
    }
  }
  if (!detail::Finite(candidate.rotation) || !detail::Finite(candidate.frame) || !detail::Finite(candidate.spin))
    return Status::kNonfiniteResult;
  output = candidate;
  return Status::kSuccess;
}

// weighted_frozen ALREADY includes shape weights N_n. Return complete weighted
// derivatives W*_n = W_n + (I - sum_m W_m) B_n. In particular, zero ANS shape
// weight does not imply a zero completed derivative. Never divide by N_n or
// multiply the returned derivative by N_n again. All inputs must describe the
// same current configuration and WORLD convention.
TL_REISSNER_HD inline Status CorrectOrientationVariation(const SpinJacobian& weighted_frozen,
                                                         const SpinJacobian& mean_spin,
                                                         SpinJacobian& output) {
  const Matrix3 identity{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  return detail::CompleteVariation(identity, weighted_frozen, mean_spin, output);
}

// Complete one WORLD spatial curvature derivative:
// C*_n = C_n + (-[k]_x - sum_m C_m) B_n, k = A DRot(phi) phi_,alpha.
// This does not convert to material-frame curvature. Differentiating T^T k
// also requires the completed orientation variation returned above.
TL_REISSNER_HD inline Status CorrectCurvatureVariation(Vec3 spatial_curvature,
                                                      const SpinJacobian& frozen,
                                                      const SpinJacobian& mean_spin,
                                                      SpinJacobian& output) {
  const Matrix3 negative_star{{0, spatial_curvature.z, -spatial_curvature.y,
                              -spatial_curvature.z, 0, spatial_curvature.x,
                              spatial_curvature.y, -spatial_curvature.x, 0}};
  return detail::CompleteVariation(negative_star, frozen, mean_spin, output);
}

}  // namespace tl::fea::reissner

#undef TL_REISSNER_HD
