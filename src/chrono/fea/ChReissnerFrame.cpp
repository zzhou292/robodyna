// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
// =============================================================================

#include "chrono/fea/ChReissnerFrame.h"
#include "chrono/core/ChMatrixMBD.h"

#include <cmath>

namespace chrono {
namespace fea {
namespace {

bool Finite(const ChQuaterniond& q) {
    for (unsigned i = 0; i < 4; ++i)
        if (!std::isfinite(q.data()[i]))
            return false;
    return true;
}

bool Finite(const ChReissnerSpinJacobian& matrices) {
    for (const auto& matrix : matrices)
        if (!matrix.allFinite())
            return false;
    return true;
}

ChReissnerFrameStatus CompleteVariation(const ChMatrix33<>& common_spin,
                                       const ChReissnerSpinJacobian& frozen,
                                       const ChReissnerSpinJacobian& mean_spin,
                                       ChReissnerSpinJacobian& output) {
    if (!common_spin.allFinite() || !Finite(frozen) || !Finite(mean_spin))
        return ChReissnerFrameStatus::NonfiniteInput;
    ChMatrix33<> closure = common_spin;
    for (const auto& matrix : frozen)
        closure -= matrix;
    if (!closure.allFinite())
        return ChReissnerFrameStatus::NonfiniteResult;
    ChReissnerSpinJacobian candidate;
    for (unsigned n = 0; n < 4; ++n)
        candidate[n] = frozen[n] + closure * mean_spin[n];
    if (!Finite(candidate))
        return ChReissnerFrameStatus::NonfiniteResult;
    output = candidate;
    return ChReissnerFrameStatus::Success;
}

}  // namespace

ChReissnerFrameStatus ComputeReissnerMeanFrame(const std::array<ChQuaterniond, 4>& rotation,
                                              ChReissnerMeanFrame& output) {
    constexpr double unit_tolerance = 1e-12;
    constexpr double minimum_pair_dot = 0.70710678118654752440;  // cos(pi/4)
    auto aligned = rotation;
    for (auto& q : aligned) {
        if (!Finite(q))
            return ChReissnerFrameStatus::NonfiniteInput;
        const double norm_squared = q ^ q;
        if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1) > unit_tolerance)
            return ChReissnerFrameStatus::NonUnitQuaternion;
        if ((q ^ rotation[0]) < 0)
            q *= -1;
    }
    for (unsigned n = 0; n < 4; ++n)
        for (unsigned m = 0; m < n; ++m)
            if (!((aligned[n] ^ aligned[m]) > minimum_pair_dot))
                return ChReissnerFrameStatus::OutsideChart;

    ChQuaterniond sum(0, 0, 0, 0);
    for (const auto& q : aligned)
        sum += q * .25;
    const double length = std::sqrt(sum ^ sum);
    // The pairwise chart analytically excludes a vanishing sum. Keep this
    // runtime check rather than relying on Quaternion::Normalize's fallback.
    if (!std::isfinite(length) || length <= .5)
        return ChReissnerFrameStatus::NonfiniteResult;
    ChReissnerMeanFrame candidate;
    candidate.rotation = sum * (1 / length);
    candidate.frame.SetFromQuaternion(candidate.rotation);
    const auto conjugate = candidate.rotation.GetConjugate();
    for (unsigned n = 0; n < 4; ++n) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            ChVector3d direction(0, 0, 0);
            direction[axis] = 1;
            const auto d_sum = QuatDtFromAngVelAbs(direction, aligned[n]) * .25;
            const auto d_mean = (d_sum - candidate.rotation * (candidate.rotation ^ d_sum)) * (1 / length);
            const auto spin = Qcross(d_mean, conjugate) * 2;
            candidate.spin[n](0, axis) = spin.e1();
            candidate.spin[n](1, axis) = spin.e2();
            candidate.spin[n](2, axis) = spin.e3();
        }
    }
    if (!Finite(candidate.rotation) || !candidate.frame.allFinite() || !Finite(candidate.spin))
        return ChReissnerFrameStatus::NonfiniteResult;
    output = candidate;
    return ChReissnerFrameStatus::Success;
}

ChReissnerFrameStatus CorrectReissnerOrientationVariation(const ChReissnerSpinJacobian& weighted_frozen,
                                                          const ChReissnerSpinJacobian& mean_spin,
                                                          ChReissnerSpinJacobian& output) {
    return CompleteVariation(ChMatrix33<>(1), weighted_frozen, mean_spin, output);
}

ChReissnerFrameStatus CorrectReissnerCurvatureVariation(const ChVector3d& spatial_curvature,
                                                       const ChReissnerSpinJacobian& frozen,
                                                       const ChReissnerSpinJacobian& mean_spin,
                                                       ChReissnerSpinJacobian& output) {
    // delta(theta) x k = -[k]_x delta(theta).
    return CompleteVariation(-ChStarMatrix33<>(spatial_curvature), frozen, mean_spin, output);
}

}  // namespace fea
}  // namespace chrono
