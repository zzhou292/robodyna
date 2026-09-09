#include "ReissnerGaussContractFixture.h"

#include <cstring>
#include <limits>

namespace crash::qualification {
namespace {
void Near(double actual, double expected) { EXPECT_NEAR(actual, expected, 3e-12 * (1 + std::abs(expected))); }
void VectorNear(tl_shell::Vec3 actual, const Vec& expected) {
    for (unsigned c = 0; c < 3; ++c) Near(tl_shell::detail::Component(actual, c), expected[c]);
}
}

void ReissnerGaussContract::CheckPointIntermediatesAndContraction(const Frames& frames) {
    ConfigurePoints(frames); ASSERT_FALSE(HasFatalFailure());
    for (unsigned p = 0; p < 4; ++p) {
        for (unsigned r = 0; r < 12; ++r) points.resultant[p][r] = ((r % 2) ? -.3 : .7) * (r + 1) / (p + 2);
        for (unsigned c = 0; c < 24; ++c) points.force[p][c] = .013 * (c + 1);
    }
    const auto initial = points; RunPoints(); ASSERT_FALSE(HasFatalFailure());
    EXPECT_TRUE(Evaluate(frames).finite);  // Refresh actual public Chrono point data.
    tl_shell::shell_detail::Kinematics state;
    ASSERT_EQ(tl_shell::shell_detail::PrepareKinematics(points.reference, points.configuration, state), tl_shell::Status::kSuccess);
    for (unsigned p = 0; p < 4; ++p) {
        ASSERT_EQ(points.status[p], tl_shell::Status::kSuccess);
        const auto& q = points.reference.gauss[p]; const auto& compact = points.point[p];
        gauss_contract::GaussPointKinematics host;
        ASSERT_EQ(gauss_contract::PrepareGaussPoint(state, points.configuration, q, host), tl_shell::Status::kSuccess);
        tl_shell::shell_detail::PointResponse scalar;
        ASSERT_EQ(tl_shell::shell_detail::EvaluatePoint(state, points.configuration, q, true, scalar), tl_shell::Status::kSuccess);
        for (unsigned r = 0; r < 12; ++r) { Near(compact.strain[r], host.strain[r]); Near(compact.strain[r], scalar.strain[r]); }
        for (unsigned i = 0; i < 9; ++i) {
            Near(compact.frame_transpose.v[i], element->T_i[p](i % 3, i / 3));
            Near(compact.world_gamma.v[i], host.world_gamma.v[i]);
        }
        VectorNear(compact.phi, element->phi_tilde_i[p]);
        VectorNear(compact.position_gradient[0], element->y_i_1[p]);
        VectorNear(compact.position_gradient[1], element->y_i_2[p]);
        VectorNear(compact.spatial_curvature[0], element->k_1_i[p]);
        VectorNear(compact.spatial_curvature[1], element->k_2_i[p]);
        const double weight[2][4] = {{(1 + q.natural[1]) * .5, 0, (1 - q.natural[1]) * .5, 0},
                                     {0, (1 - q.natural[0]) * .5, 0, (1 + q.natural[0]) * .5}};
        for (unsigned axis = 0; axis < 2; ++axis) for (unsigned c = 0; c < 24; ++c) {
            scalar.derivative[3 * axis + 2][c] = 0;
            for (unsigned a = 0; a < 4; ++a)
                scalar.derivative[3 * axis + 2][c] += weight[axis][a] * points.ans_derivative[a][axis][c];
        }
        long double force_work = 0, strain_work = 0;
        for (unsigned c = 0; c < 24; ++c) {
            long double force = initial.force[p][c];
            for (unsigned r = 0; r < 12; ++r)
                force -= static_cast<long double>(q.area_weight) * scalar.derivative[r][c] * points.resultant[p][r];
            Near(points.force[p][c], static_cast<double>(force));
            const double variation = ((c % 2) ? -.17 : .23) * (c + 1);
            force_work += (points.force[p][c] - initial.force[p][c]) * variation;
            for (unsigned r = 0; r < 12; ++r)
                strain_work += static_cast<long double>(q.area_weight) * points.resultant[p][r] * scalar.derivative[r][c] * variation;
        }
        EXPECT_NEAR(static_cast<double>(force_work + strain_work), 0, 3e-12 * (1 + std::abs(static_cast<double>(strain_work))));
    }
}

void ReissnerGaussContract::CheckPointDerivatives(const Frames& frames) {
    ConfigurePoints(frames); ASSERT_FALSE(HasFatalFailure());
    double derivative[4][12][24]{};
    for (unsigned row = 0; row < 12; ++row) {
        for (unsigned p = 0; p < 4; ++p) {
            for (auto& value : points.resultant[p]) value = 0;
            for (auto& value : points.force[p]) value = 0;
            points.resultant[p][row] = 1;
        }
        RunPoints(); ASSERT_FALSE(HasFatalFailure());
        for (unsigned p = 0; p < 4; ++p) {
            ASSERT_EQ(points.status[p], tl_shell::Status::kSuccess);
            for (unsigned c = 0; c < 24; ++c) derivative[p][row][c] = -points.force[p][c] / points.reference.gauss[p].area_weight;
        }
    }
    // Independent actual-Chrono strain values. Unit resultant probes recover
    // every new force column without asking the new operation for a B matrix.
    // Same fixed FD/halving budgets as stage 1; curvature adds its 1/metre scale.
    for (unsigned node = 0; node < 4; ++node) for (unsigned dof = 0; dof < 6; ++dof) {
        SCOPED_TRACE(node);
        SCOPED_TRACE(dof);
        Vec direction(0, 0, 0); direction[dof % 3] = 1;
        double maximum_error[2]{}; unsigned level = 0;
        for (double h : {2e-5, 1e-5}) {
            Evaluation value[2];
            for (unsigned sign = 0; sign < 2; ++sign) {
                auto changed = frames; const double delta = sign ? h : -h;
                if (dof < 3) changed.x[node][dof] += delta;
                else changed.q[node] = Rotation(delta, direction) * frames.q[node];
                value[sign] = Evaluate(changed); ASSERT_TRUE(value[sign].finite);
            }
            for (unsigned p = 0; p < 4; ++p) for (unsigned row = 0; row < 12; ++row) {
                const double fd = (value[1].strain[p][row] - value[0].strain[p][row]) / (2 * h);
                const double expected = derivative[p][row][6 * node + dof];
                const double scale = (dof < 3 ? 1 / kLength : 1) * (row < 6 ? 1 : 1 / kLength);
                const double error = std::abs(fd - expected);
                EXPECT_LE(error, 1e-8 * scale + 2e-7 * std::max(std::abs(fd), std::abs(expected)));
                maximum_error[level] = std::max(maximum_error[level], error / scale);
            }
            ++level;
        }
        EXPECT_LE(maximum_error[1], .35 * maximum_error[0] + 2e-9);
    }
}

TEST_F(ReissnerGaussContract, CompactGaussIntermediatesAndArbitraryResultantVirtualWorkMatchOracles) {
    CheckPointIntermediatesAndContraction(GeneralShellDeformation());
    CheckPointIntermediatesAndContraction(Transform(GeneralShellDeformation(), Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7)));
    ReferenceUnchanged();
}
TEST_F(ReissnerGaussContract, EveryGaussForceColumnDifferentiatesActualChronoInAllWorldDofs) {
    CheckPointDerivatives(GeneralShellDeformation());
    CheckPointDerivatives(Transform(GeneralShellDeformation(), Rotation(1.7, Vec(1, 2, -1)), Vec(2.1, -1.3, .7)));
    ReferenceUnchanged();
}
TEST_F(ReissnerGaussContract, LatePointAndContractionFailuresPreserveCompleteAccumulatorsAndRetry) {
    ConfigurePoints(GeneralShellDeformation());
    for (unsigned p = 0; p < 4; ++p) {
        for (auto& value : points.resultant[p]) value = .3;
        for (auto& value : points.force[p]) value = .7;
    }
    const auto initial = points; RunPoints(); const auto clean = points;
    points = initial;
    points.reference.gauss[3].gradient[0][0] = std::numeric_limits<double>::max();
    std::memset(&points.point[3], 0x35, sizeof(points.point[3])); const auto invalid = points;
    RunPoints();
    for (unsigned p = 0; p < 3; ++p) EXPECT_EQ(points.status[p], tl_shell::Status::kSuccess);
    EXPECT_EQ(points.status[3], tl_shell::Status::kNonfiniteResult);
    EXPECT_EQ(std::memcmp(&points.point[3], &invalid.point[3], sizeof(points.point[3])), 0);
    EXPECT_EQ(std::memcmp(points.force[3], invalid.force[3], sizeof(points.force[3])), 0);
    points = initial;
    // Valid point kinematics; only the last node's last ANS column overflows
    // during contraction, after earlier local force entries were evaluated.
    points.ans_derivative[3][1][23] = std::numeric_limits<double>::max();
    for (unsigned p = 0; p < 4; ++p) points.resultant[p][5] = 64;
    const auto overflow = points; RunPoints();
    for (unsigned p = 0; p < 4; ++p) {
        EXPECT_EQ(points.status[p], tl_shell::Status::kNonfiniteResult);
        EXPECT_EQ(std::memcmp(points.force[p], overflow.force[p], sizeof(points.force[p])), 0);
    }
    tl_shell::shell_detail::Kinematics state;
    ASSERT_EQ(tl_shell::shell_detail::PrepareKinematics(overflow.reference, overflow.configuration, state), tl_shell::Status::kSuccess);
    double host_force[24]; std::memcpy(host_force, overflow.force[3], sizeof(host_force));
    EXPECT_EQ(gauss_contract::AccumulateGaussPointForce(state, overflow.reference.gauss[3], clean.point[3],
        overflow.ans_derivative, overflow.resultant[3], host_force), tl_shell::Status::kNonfiniteResult);
    EXPECT_EQ(std::memcmp(host_force, overflow.force[3], sizeof(host_force)), 0);
    points = initial; RunPoints();
    for (unsigned p = 0; p < 4; ++p) EXPECT_EQ(points.status[p], tl_shell::Status::kSuccess);
    EXPECT_EQ(std::memcmp(points.force, clean.force, sizeof(points.force)), 0);
}
}  // namespace crash::qualification
