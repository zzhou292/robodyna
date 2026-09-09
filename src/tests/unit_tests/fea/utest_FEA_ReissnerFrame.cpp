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

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
using namespace chrono;
using namespace chrono::fea;
using Status = ChReissnerFrameStatus;
using Rotations = std::array<ChQuaterniond, 4>;
using Mat = ChMatrix33<>;
using Vec = ChVector3d;

ChQuaterniond Rotation(const Vec& vector) {
    ChQuaterniond q;
    q.SetFromRotVec(vector);
    return q;
}

Rotations Noncoaxial() {
    return {{Rotation({.11, .03, -.07}), Rotation({-.08, .14, .04}),
             Rotation({.02, -.06, .16}), Rotation({-.05, -.04, -.1})}};
}

Rotations Perturb(Rotations rotations, unsigned node, unsigned axis, double amount) {
    Vec vector(0, 0, 0);
    vector[axis] = amount;
    rotations[node] = Rotation(vector) * rotations[node];  // Independent left/world increment.
    return rotations;
}

void MatrixNear(const Mat& actual, const Mat& expected, double tolerance) {
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col)
            EXPECT_NEAR(actual(row, col), expected(row, col), tolerance) << row << ',' << col;
}

Mat Sum(const ChReissnerSpinJacobian& matrices) {
    Mat sum(0);
    for (const auto& matrix : matrices)
        sum += matrix;
    return sum;
}

Vec Vee(const Mat& matrix) {
    return {.5 * (matrix(2, 1) - matrix(1, 2)), .5 * (matrix(0, 2) - matrix(2, 0)),
            .5 * (matrix(1, 0) - matrix(0, 1))};
}

ChReissnerMeanFrame Mean(const Rotations& rotations) {
    ChReissnerMeanFrame mean;
    EXPECT_EQ(ComputeReissnerMeanFrame(rotations, mean), Status::Success);
    return mean;
}

void SameMean(const ChReissnerMeanFrame& a, const ChReissnerMeanFrame& b) {
    for (unsigned i = 0; i < 4; ++i)
        EXPECT_DOUBLE_EQ(a.rotation.data()[i], b.rotation.data()[i]);
    MatrixNear(a.frame, b.frame, 0);
    for (unsigned i = 0; i < 4; ++i)
        MatrixNear(a.spin[i], b.spin[i], 0);
}

TEST(ReissnerFrame, UniformDirectorsHaveKnownMeanAndQuarterSpinJacobians) {
    for (const auto& spin : {Vec(0, 0, 0), Vec(.8, -.6, 1.2), Vec(2.8, 0, 0)}) {
        const auto q = Rotation(spin);
        const auto mean = Mean({{q, q, q, q}});
        MatrixNear(mean.frame, Mat(q), 8e-15);
        MatrixNear(mean.frame.transpose() * mean.frame, Mat(1), 8e-15);
        EXPECT_NEAR(mean.frame.determinant(), 1, 8e-15);
        for (const auto& jacobian : mean.spin)
            MatrixNear(jacobian, Mat(.25), 4e-15);
    }
}

TEST(ReissnerFrame, MeanEachNodeSpinMatchesCenteredDerivativeAndRefinement) {
    const auto rotations = Noncoaxial();
    const auto mean = Mean(rotations);
    double previous_error = 0;
    for (double delta : {1e-3, 5e-4, 2.5e-4}) {
        double maximum_error = 0;
        for (unsigned node = 0; node < 4; ++node) {
            for (unsigned axis = 0; axis < 3; ++axis) {
                const auto plus = Mean(Perturb(rotations, node, axis, delta));
                const auto minus = Mean(Perturb(rotations, node, axis, -delta));
                const Vec derivative = Vee(((plus.frame - minus.frame) / (2 * delta)) * mean.frame.transpose());
                for (unsigned component = 0; component < 3; ++component)
                    maximum_error = std::max(maximum_error, std::abs(derivative[component] - mean.spin[node](component, axis)));
            }
        }
        EXPECT_LT(maximum_error, 8e-8);
        if (previous_error > 0)
            EXPECT_LT(maximum_error, .35 * previous_error);  // Centered O(delta^2), not a fitted value oracle.
        previous_error = maximum_error;
    }
    MatrixNear(Sum(mean.spin), Mat(1), 4e-15);
}

TEST(ReissnerFrame, SignsPermutationsAndLargeCommonRotationPreservePhysicalMeanAndSpin) {
    const auto rotations = Noncoaxial();
    const auto mean = Mean(rotations);
    for (unsigned signs = 0; signs < 16; ++signs) {
        auto changed = rotations;
        for (unsigned n = 0; n < 4; ++n)
            if (signs & (1u << n))
                changed[n] *= -1;
        const auto result = Mean(changed);
        MatrixNear(result.frame, mean.frame, 3e-15);
        for (unsigned n = 0; n < 4; ++n)
            MatrixNear(result.spin[n], mean.spin[n], 3e-15);
    }
    std::array<unsigned, 4> permutation{{0, 1, 2, 3}};
    do {
        Rotations changed;
        for (unsigned n = 0; n < 4; ++n)
            changed[n] = rotations[permutation[n]];
        const auto result = Mean(changed);
        MatrixNear(result.frame, mean.frame, 3e-15);
        for (unsigned n = 0; n < 4; ++n)
            MatrixNear(result.spin[n], mean.spin[permutation[n]], 3e-15);
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    for (const auto& spin : {Vec(.8, -.6, 1.2), Vec(2.8, 0, 0), Vec(0, 0, 3.141592653589793)}) {
        const auto q = Rotation(spin);
        const Mat rigid(q);
        auto changed = rotations;
        for (auto& rotation : changed)
            rotation = q * rotation;
        const auto result = Mean(changed);
        MatrixNear(result.frame, rigid * mean.frame, 5e-15);
        for (unsigned n = 0; n < 4; ++n)
            MatrixNear(result.spin[n], rigid * mean.spin[n] * rigid.transpose(), 5e-15);
    }
}

TEST(ReissnerFrame, InvalidQuaternionOrRelativeChartLeavesAllMeanOutputUnchanged) {
    const auto valid = Noncoaxial();
    auto output = Mean(valid);
    const auto sentinel = output;
    for (unsigned variant = 0; variant < 6; ++variant) {
        auto rotations = valid;
        Status expected = Status::NonUnitQuaternion;
        if (variant == 0)
            rotations[3] = {0, 0, 0, 0};
        if (variant == 1)
            rotations[2] *= 1.00001;
        if (variant == 2)
            rotations[1] = {1e300, 0, 0, 0};
        if (variant == 3) {
            rotations[2].e1() = std::numeric_limits<double>::quiet_NaN();
            expected = Status::NonfiniteInput;
        }
        if (variant == 4) {
            rotations[3] = Rotation({2, 0, 0});
            expected = Status::OutsideChart;
        }
        if (variant == 5) {
            // Each director is within 90 degrees of the anchor, but the two
            // opposite rotations violate the required pairwise chart.
            rotations = {{Rotation({0, 0, 0}), Rotation({1, 0, 0}), Rotation({-1, 0, 0}), Rotation({0, 0, 0})}};
            expected = Status::OutsideChart;
        }
        EXPECT_EQ(ComputeReissnerMeanFrame(rotations, output), expected);
        SameMean(output, sentinel);
    }
    EXPECT_EQ(ComputeReissnerMeanFrame(valid, output), Status::Success);
    SameMean(output, sentinel);
}

struct Interpolation {
    std::array<double, 4> shape;
    std::array<double, 4> derivative;
};

struct PointValues {
    Mat rotation;
    Vec curvature;
};

// Independent VALUE oracle: quaternion log/exp and the closed-form spatial
// derivative of Rodrigues' exponential. No ChRotUtils, mean Jacobian, or
// correction formula is used to differentiate these values numerically below.
PointValues Values(const Rotations& nodal, const ChQuaterniond& mean, const Interpolation& interpolation) {
    Vec phi(0, 0, 0), gradient(0, 0, 0);
    for (unsigned n = 0; n < 4; ++n) {
        auto relative = mean.GetConjugate() * nodal[n];
        if (relative.e0() < 0)
            relative *= -1;
        const auto logarithm = relative.GetRotVec();
        phi += interpolation.shape[n] * logarithm;
        gradient += interpolation.derivative[n] * logarithm;
    }
    const double angle = phi.Length(), square = angle * angle;
    double a, b;
    if (angle < 1e-3) {
        a = .5 - square / 24 + square * square / 720;
        b = 1. / 6 - square / 120 + square * square / 5040;
    } else {
        a = 2 * std::pow(std::sin(angle / 2) / angle, 2);
        b = (1 - std::sin(angle) / angle) / square;
    }
    return {Mat(mean * Rotation(phi)), Mat(mean) * (gradient + a * Vcross(phi, gradient) + b * Vcross(phi, Vcross(phi, gradient)))};
}

ChReissnerSpinJacobian NumericalDerivative(const Rotations& rotations,
                                          const ChReissnerMeanFrame& mean,
                                          const Interpolation& interpolation,
                                          bool curvature,
                                          bool moving_mean,
                                          double delta) {
    ChReissnerSpinJacobian result;
    const auto base = Values(rotations, mean.rotation, interpolation);
    for (unsigned n = 0; n < 4; ++n) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            const auto plus_nodes = Perturb(rotations, n, axis, delta);
            const auto minus_nodes = Perturb(rotations, n, axis, -delta);
            const auto plus = Values(plus_nodes, moving_mean ? Mean(plus_nodes).rotation : mean.rotation, interpolation);
            const auto minus = Values(minus_nodes, moving_mean ? Mean(minus_nodes).rotation : mean.rotation, interpolation);
            const Vec column = curvature ? (plus.curvature - minus.curvature) / (2 * delta) :
                Vee(((plus.rotation - minus.rotation) / (2 * delta)) * base.rotation.transpose());
            for (unsigned row = 0; row < 3; ++row)
                result[n](row, axis) = column[row];
        }
    }
    return result;
}

const std::array<Interpolation, 3> locations{{
    {{{.28, .42, .18, .12}}, {{-.35, .35, .15, -.15}}},  // Interior Q4 point, xi derivative.
    {{{.5, .5, 0, 0}}, {{-.5, .5, 0, 0}}},             // ANS edge, xi derivative.
    {{{.5, .5, 0, 0}}, {{-.25, -.25, .25, .25}}}       // Same ANS edge, eta derivative.
}};

TEST(ReissnerFrame, WeightedOrientationCorrectionMatchesFullDerivativeIncludingZeroAnsWeights) {
    for (const auto& common_spin : {Vec(0, 0, 0), Vec(1.2, -.8, .9)}) {
        auto rotations = Noncoaxial();
        for (auto& q : rotations)
            q = Rotation(common_spin) * q;
        const auto mean = Mean(rotations);
        for (const auto& location : locations) {
            const auto frozen = NumericalDerivative(rotations, mean, location, false, false, 1e-4);
            const auto independent = NumericalDerivative(rotations, mean, location, false, true, 5e-5);
            ChReissnerSpinJacobian corrected;
            ASSERT_EQ(CorrectReissnerOrientationVariation(frozen, mean.spin, corrected), Status::Success);
            for (unsigned n = 0; n < 4; ++n)
                MatrixNear(corrected[n], independent[n], 2e-8);
            MatrixNear(Sum(corrected), Mat(1), 5e-15);
            if (location.shape[2] == 0) {
                EXPECT_LT(frozen[2].norm(), 1e-11);
                EXPECT_GT(corrected[2].norm(), 1e-6);  // Mean coupling cannot be removed by multiplying N_2=0.
            }
        }
    }
}

TEST(ReissnerFrame, SpatialCurvatureCorrectionMatchesFullDerivativeAndRigidSpin) {
    for (const auto& common_spin : {Vec(0, 0, 0), Vec(1.2, -.8, .9)}) {
        auto rotations = Noncoaxial();
        for (auto& q : rotations)
            q = Rotation(common_spin) * q;
        const auto mean = Mean(rotations);
        for (const auto& location : locations) {
            const auto values = Values(rotations, mean.rotation, location);
            ASSERT_GT(values.curvature.Length(), .01);
            const auto frozen = NumericalDerivative(rotations, mean, location, true, false, 1e-4);
            const auto independent = NumericalDerivative(rotations, mean, location, true, true, 5e-5);
            ChReissnerSpinJacobian corrected;
            ASSERT_EQ(CorrectReissnerCurvatureVariation(values.curvature, frozen, mean.spin, corrected), Status::Success);
            for (unsigned n = 0; n < 4; ++n)
                MatrixNear(corrected[n], independent[n], 2e-8);
            MatrixNear(Sum(corrected), -ChStarMatrix33<>(values.curvature), 8e-15);
        }
    }
}

TEST(ReissnerFrame, CorrectionsAllowAliasedArraysAndRejectNonfiniteOrOverflowingResultsTransactionally) {
    const auto mean = Mean(Noncoaxial());
    ChReissnerSpinJacobian frozen{{Mat(.1), Mat(.2), Mat(.3), Mat(.15)}}, expected;
    ASSERT_EQ(CorrectReissnerOrientationVariation(frozen, mean.spin, expected), Status::Success);
    auto aliased = frozen;
    ASSERT_EQ(CorrectReissnerOrientationVariation(aliased, mean.spin, aliased), Status::Success);
    for (unsigned n = 0; n < 4; ++n)
        MatrixNear(aliased[n], expected[n], 0);
    auto aliased_mean = mean.spin;
    ASSERT_EQ(CorrectReissnerOrientationVariation(frozen, aliased_mean, aliased_mean), Status::Success);
    for (unsigned n = 0; n < 4; ++n)
        MatrixNear(aliased_mean[n], expected[n], 0);
    const auto sentinel = expected;
    auto bad = frozen;
    bad[3](2, 2) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(CorrectReissnerOrientationVariation(bad, mean.spin, expected), Status::NonfiniteInput);
    for (unsigned n = 0; n < 4; ++n)
        MatrixNear(expected[n], sentinel[n], 0);
    EXPECT_EQ(CorrectReissnerCurvatureVariation({0, std::numeric_limits<double>::infinity(), 0}, frozen, mean.spin, expected),
              Status::NonfiniteInput);
    for (unsigned n = 0; n < 4; ++n)
        MatrixNear(expected[n], sentinel[n], 0);
    for (auto& matrix : bad)
        matrix = Mat(1e308);
    EXPECT_EQ(CorrectReissnerOrientationVariation(bad, mean.spin, expected), Status::NonfiniteResult);
    for (unsigned n = 0; n < 4; ++n)
        MatrixNear(expected[n], sentinel[n], 0);
    bad = frozen;
    auto large_mean = mean.spin;
    large_mean[3] = Mat(1e308);
    bad[0] = Mat(-4);
    EXPECT_EQ(CorrectReissnerCurvatureVariation({.1, .2, -.3}, bad, large_mean, expected), Status::NonfiniteResult);
    for (unsigned n = 0; n < 4; ++n)
        MatrixNear(expected[n], sentinel[n], 0);
    EXPECT_EQ(CorrectReissnerOrientationVariation(frozen, mean.spin, expected), Status::Success);
}
}  // namespace
