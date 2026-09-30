// Reissner frame primitives only: owning Chrono CPU implementation versus the
// actual TL CUDA operation, plus independent quaternion value derivatives.
// The value oracle follows the focused Chrono frame tests (BSD-3-Clause;
// chrono/LICENSE), without reusing the analytic Jacobian/correction arithmetic.
#include "lib_src/elements/ReissnerFrame.h"
#include "chrono/fea/ChReissnerFrame.h"
#include "chrono/core/ChMatrixMBD.h"

#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
namespace gpu = tl::fea::reissner;
namespace cpu = chrono::fea;
using Vec = chrono::ChVector3d;
using Mat = chrono::ChMatrix33<>;
using Quat = chrono::ChQuaterniond;
using Rotations = std::array<Quat, 4>;
using Spin = cpu::ChReissnerSpinJacobian;
using CpuStatus = cpu::ChReissnerFrameStatus;

// Predeclared binary64 budgets: direct arithmetic agreement and independent
// centered derivatives are separate comparisons. No fast-math/FMA contraction.
constexpr double kDirect = 2e-14;
constexpr double kDerivative = 2e-8;

enum class Operation { Mean, Orientation, Curvature };
struct Packet {
    gpu::Quaternion rotation[4]{};
    gpu::MeanFrame mean{};
    gpu::SpinJacobian frozen{}, spin{}, output{};
    gpu::Vec3 curvature{};
    gpu::Status status = gpu::Status::kSuccess;
    Operation operation = Operation::Mean;
    unsigned alias = 0;  // 0: output, 1: frozen, 2: mean-spin input.
};
static_assert(sizeof(Packet) < 4096, "The fixture must remain a tiny allocation");

__global__ void Evaluate(Packet* p) {
    if (blockIdx.x || threadIdx.x) return;
    if (p->operation == Operation::Mean) {
        p->status = gpu::ComputeMeanFrame(p->rotation, p->mean);
        return;
    }
    auto& output = p->alias == 1 ? p->frozen : (p->alias == 2 ? p->spin : p->output);
    p->status = p->operation == Operation::Orientation
                    ? gpu::CorrectOrientationVariation(p->frozen, p->spin, output)
                    : gpu::CorrectCurvatureVariation(p->curvature, p->frozen, p->spin, output);
}

class ReissnerFrameCuda : public ::testing::Test {
  protected:
    Packet* device_ = nullptr;
    void SetUp() override {
        int count = 0;
        ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess);
        ASSERT_GT(count, 0);  // A missing/broken GPU is failure, never a skipped gate.
        ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device_), sizeof(Packet)), cudaSuccess);
    }
    void TearDown() override {
        if (device_) EXPECT_EQ(cudaFree(device_), cudaSuccess);
    }
    ::testing::AssertionResult Run(Packet& packet) {
        auto error = cudaMemcpy(device_, &packet, sizeof(Packet), cudaMemcpyHostToDevice);
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        Evaluate<<<1, 1>>>(device_);
        error = cudaGetLastError();
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        error = cudaDeviceSynchronize();
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        error = cudaMemcpy(&packet, device_, sizeof(Packet), cudaMemcpyDeviceToHost);
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        return ::testing::AssertionSuccess();
    }
};

Quat Rotation(const Vec& vector) {
    Quat q;
    q.SetFromRotVec(vector);
    return q;
}
Rotations Noncoaxial() {
    return {{Rotation({.11, .03, -.07}), Rotation({-.08, .14, .04}),
             Rotation({.02, -.06, .16}), Rotation({-.05, -.04, -.1})}};
}
Rotations CommonRotation(Rotations rotations, const Vec& spin) {
    const auto q = Rotation(spin);
    for (auto& node : rotations) node = q * node;
    return rotations;
}
Rotations Perturb(Rotations rotations, unsigned node, unsigned axis, double amount) {
    Vec spin(0, 0, 0);
    spin[axis] = amount;
    rotations[node] = Rotation(spin) * rotations[node];
    return rotations;
}
void Bind(Packet& p, const Rotations& rotations) {
    for (unsigned n = 0; n < 4; ++n)
        p.rotation[n] = {rotations[n].e0(), rotations[n].e1(), rotations[n].e2(), rotations[n].e3()};
}
gpu::SpinJacobian ToGpu(const Spin& source) {
    gpu::SpinJacobian result{};
    for (unsigned n = 0; n < 4; ++n)
        for (unsigned r = 0; r < 3; ++r)
            for (unsigned c = 0; c < 3; ++c) result.node[n].v[3 * r + c] = source[n](r, c);
    return result;
}
Mat ToCpu(const gpu::Matrix3& source) {
    Mat result(0);
    for (unsigned r = 0; r < 3; ++r)
        for (unsigned c = 0; c < 3; ++c) result(r, c) = source.v[3 * r + c];
    return result;
}
void Near(const gpu::Matrix3& actual, const Mat& expected, double tolerance = kDirect) {
    for (unsigned r = 0; r < 3; ++r)
        for (unsigned c = 0; c < 3; ++c)
            EXPECT_NEAR(actual.v[3 * r + c], expected(r, c), tolerance) << r << ',' << c;
}
void Near(const gpu::SpinJacobian& actual, const Spin& expected, double tolerance = kDirect) {
    for (unsigned n = 0; n < 4; ++n) {
        SCOPED_TRACE(n);
        Near(actual.node[n], expected[n], tolerance);
    }
}
void Near(const gpu::MeanFrame& actual, const cpu::ChReissnerMeanFrame& expected) {
    const double dot = actual.rotation.w * expected.rotation.e0() + actual.rotation.x * expected.rotation.e1() +
                       actual.rotation.y * expected.rotation.e2() + actual.rotation.z * expected.rotation.e3();
    const double sign = dot < 0 ? -1 : 1;
    EXPECT_NEAR(actual.rotation.w * sign, expected.rotation.e0(), kDirect);
    EXPECT_NEAR(actual.rotation.x * sign, expected.rotation.e1(), kDirect);
    EXPECT_NEAR(actual.rotation.y * sign, expected.rotation.e2(), kDirect);
    EXPECT_NEAR(actual.rotation.z * sign, expected.rotation.e3(), kDirect);
    Near(actual.frame, expected.frame);
    Near(actual.spin, expected.spin);
}
void Unchanged(const gpu::SpinJacobian& actual, const gpu::SpinJacobian& previous) {
    // Compare each double's bits, including NaN payloads in aliased bad inputs;
    // do not compare unspecified struct padding.
    for (unsigned n = 0; n < 4; ++n)
        for (unsigned i = 0; i < 9; ++i)
            EXPECT_EQ(std::memcmp(&actual.node[n].v[i], &previous.node[n].v[i], sizeof(double)), 0);
}
void Unchanged(const gpu::MeanFrame& actual, const gpu::MeanFrame& previous) {
    const double a[] = {actual.rotation.w, actual.rotation.x, actual.rotation.y, actual.rotation.z};
    const double b[] = {previous.rotation.w, previous.rotation.x, previous.rotation.y, previous.rotation.z};
    for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(std::memcmp(&a[i], &b[i], sizeof(double)), 0);
    for (unsigned i = 0; i < 9; ++i)
        EXPECT_EQ(std::memcmp(&actual.frame.v[i], &previous.frame.v[i], sizeof(double)), 0);
    Unchanged(actual.spin, previous.spin);
}
cpu::ChReissnerMeanFrame CpuMean(const Rotations& rotations) {
    cpu::ChReissnerMeanFrame mean;
    EXPECT_EQ(cpu::ComputeReissnerMeanFrame(rotations, mean), CpuStatus::Success);
    return mean;
}
Mat Sum(const gpu::SpinJacobian& spin) {
    Mat result(0);
    for (const auto& node : spin.node) result += ToCpu(node);
    return result;
}

// Value-only mean, separate from both analytic implementations. The derivative
// oracle uses quaternion exp for perturbations and log of relative means.
Quat ValueMean(const Rotations& rotations) {
    Quat sum(0, 0, 0, 0);
    for (auto q : rotations) {
        if (q.Dot(rotations[0]) < 0) q *= -1;
        sum += q;
    }
    sum.Normalize();
    return sum;
}
Vec RelativeSpin(Quat plus, const Quat& minus, double delta) {
    auto relative = plus * minus.GetConjugate();
    if (relative.e0() < 0) relative *= -1;
    return relative.GetRotVec() / (2 * delta);
}

struct Interpolation { std::array<double, 4> shape, gradient; };
const std::array<Interpolation, 3> kLocations{{
    {{{.28, .42, .18, .12}}, {{-.35, .35, .15, -.15}}},
    {{{.5, .5, 0, 0}}, {{-.5, .5, 0, 0}}},
    {{{.5, .5, 0, 0}}, {{-.25, -.25, .25, .25}}}
}};
struct PointValues { Quat rotation; Vec curvature; };
PointValues Values(const Rotations& nodes, const Quat& mean, const Interpolation& point) {
    Vec phi(0, 0, 0), gradient(0, 0, 0);
    for (unsigned n = 0; n < 4; ++n) {
        auto relative = mean.GetConjugate() * nodes[n];
        if (relative.e0() < 0) relative *= -1;
        const auto logarithm = relative.GetRotVec();
        phi += point.shape[n] * logarithm;
        gradient += point.gradient[n] * logarithm;
    }
    const double angle = phi.Length(), square = angle * angle;
    const double a = angle < 1e-3 ? .5 - square / 24 + square * square / 720
                                  : 2 * std::pow(std::sin(angle / 2) / angle, 2);
    const double b = angle < 1e-3 ? 1. / 6 - square / 120 + square * square / 5040
                                  : (1 - std::sin(angle) / angle) / square;
    const Vec first = chrono::Vcross(phi, gradient);
    return {mean * Rotation(phi), Mat(mean) * (gradient + a * first + b * chrono::Vcross(phi, first))};
}
Spin Derivative(const Rotations& nodes, const Interpolation& point, bool curvature, bool moving, double delta) {
    const auto mean = ValueMean(nodes);
    Spin result;
    for (unsigned n = 0; n < 4; ++n)
        for (unsigned axis = 0; axis < 3; ++axis) {
            const auto plus_nodes = Perturb(nodes, n, axis, delta), minus_nodes = Perturb(nodes, n, axis, -delta);
            const auto plus = Values(plus_nodes, moving ? ValueMean(plus_nodes) : mean, point);
            const auto minus = Values(minus_nodes, moving ? ValueMean(minus_nodes) : mean, point);
            const Vec value = curvature ? (plus.curvature - minus.curvature) / (2 * delta)
                                        : RelativeSpin(plus.rotation, minus.rotation, delta);
            for (unsigned row = 0; row < 3; ++row) result[n](row, axis) = value[row];
        }
    return result;
}

TEST_F(ReissnerFrameCuda, OwningCpuMeanAndSpinAgreeAtNeutralAndNoncoaxialLargeCommonRotations) {
    const auto base = CpuMean(Noncoaxial());
    for (const auto& common : {Vec(0, 0, 0), Vec(.8, -.6, 1.2), Vec(2.8, 0, 0), Vec(0, 0, 3.141592653589793)}) {
        for (bool noncoaxial : {false, true}) {
            Rotations nodes = noncoaxial ? Noncoaxial() : Rotations{{Quat(1, 0, 0, 0), Quat(1, 0, 0, 0),
                                                                 Quat(1, 0, 0, 0), Quat(1, 0, 0, 0)}};
            nodes = CommonRotation(nodes, common);
            Packet p; Bind(p, nodes);
            ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
            Near(p.mean, CpuMean(nodes));
            const auto frame = ToCpu(p.mean.frame);
            EXPECT_LT((frame.transpose() * frame - Mat(1)).norm(), 3e-14);
            EXPECT_NEAR(frame.determinant(), 1, kDirect);
            EXPECT_LT((Sum(p.mean.spin) - Mat(1)).norm(), 3e-14);
            if (!noncoaxial) for (const auto& node : p.mean.spin.node) Near(node, Mat(.25));
            if (noncoaxial) {
                const Mat rigid(Rotation(common));
                Near(p.mean.frame, rigid * base.frame);
                for (unsigned n = 0; n < 4; ++n)
                    Near(p.mean.spin.node[n], rigid * base.spin[n] * rigid.transpose());
            }
        }
    }
}

TEST_F(ReissnerFrameCuda, QuaternionSignsAndEveryNodePermutationPreservePhysicalResults) {
    const auto nodes = CommonRotation(Noncoaxial(), {1.2, -.8, .9});
    const auto expected = CpuMean(nodes);
    Packet p;
    for (unsigned signs = 0; signs < 16; ++signs) {
        auto changed = nodes;
        for (unsigned n = 0; n < 4; ++n) if (signs & (1u << n)) changed[n] *= -1;
        Bind(p, changed); ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
        Near(p.mean, expected);
    }
    std::array<unsigned, 4> order{{0, 1, 2, 3}};
    do {
        Rotations changed;
        for (unsigned n = 0; n < 4; ++n) changed[n] = nodes[order[n]];
        Bind(p, changed); ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
        Near(p.mean.frame, expected.frame);
        for (unsigned n = 0; n < 4; ++n) Near(p.mean.spin.node[n], expected.spin[order[n]]);
    } while (std::next_permutation(order.begin(), order.end()));
}

TEST_F(ReissnerFrameCuda, AllTwelveWorldSpinColumnsMatchIndependentLogExpDerivativeAndRefinement) {
    for (const auto& common : {Vec(0, 0, 0), Vec(1.2, -.8, .9)}) {
        const auto nodes = CommonRotation(Noncoaxial(), common);
        Packet p; Bind(p, nodes); ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
        double previous = 0;
        for (double delta : {1e-3, 5e-4, 2.5e-4}) {
            double error = 0;
            for (unsigned n = 0; n < 4; ++n)
                for (unsigned axis = 0; axis < 3; ++axis) {
                    const Vec derivative = RelativeSpin(ValueMean(Perturb(nodes, n, axis, delta)),
                                                        ValueMean(Perturb(nodes, n, axis, -delta)), delta);
                    for (unsigned r = 0; r < 3; ++r)
                        error = std::max(error, std::abs(derivative[r] - p.mean.spin.node[n].v[3 * r + axis]));
                }
            EXPECT_LT(error, 8e-8);
            if (previous > 0) EXPECT_LT(error, .4 * previous + 2e-12);  // O(delta^2) plus binary64 roundoff.
            previous = error;
        }
    }
}

TEST_F(ReissnerFrameCuda, WeightedOrientationAgreesWithOwningCpuAndIndependentZeroAnsWeightDerivatives) {
    for (const auto& common : {Vec(0, 0, 0), Vec(1.2, -.8, .9)}) {
        const auto nodes = CommonRotation(Noncoaxial(), common);
        const auto mean = CpuMean(nodes);
        for (const auto& location : kLocations) {
            const auto frozen = Derivative(nodes, location, false, false, 1e-4);
            const auto independent = Derivative(nodes, location, false, true, 5e-5);
            Spin expected;
            ASSERT_EQ(cpu::CorrectReissnerOrientationVariation(frozen, mean.spin, expected), CpuStatus::Success);
            Packet p; p.operation = Operation::Orientation; p.frozen = ToGpu(frozen); p.spin = ToGpu(mean.spin);
            ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
            Near(p.output, expected); Near(p.output, independent, kDerivative);
            EXPECT_LT((Sum(p.output) - Mat(1)).norm(), 3e-14);
            if (location.shape[2] == 0) {
                EXPECT_LT(frozen[2].norm(), 1e-11);
                EXPECT_GT(ToCpu(p.output.node[2]).norm(), 1e-6);
            }
        }
    }
}

TEST_F(ReissnerFrameCuda, WorldCurvatureAgreesWithOwningCpuAndIndependentNoncoaxialDerivative) {
    for (const auto& common : {Vec(0, 0, 0), Vec(1.2, -.8, .9)}) {
        const auto nodes = CommonRotation(Noncoaxial(), common);
        const auto mean = CpuMean(nodes);
        for (const auto& location : kLocations) {
            const auto k = Values(nodes, ValueMean(nodes), location).curvature;
            ASSERT_GT(k.Length(), .01);
            const auto frozen = Derivative(nodes, location, true, false, 1e-4);
            const auto independent = Derivative(nodes, location, true, true, 5e-5);
            Spin expected;
            ASSERT_EQ(cpu::CorrectReissnerCurvatureVariation(k, frozen, mean.spin, expected), CpuStatus::Success);
            Packet p; p.operation = Operation::Curvature; p.frozen = ToGpu(frozen); p.spin = ToGpu(mean.spin);
            p.curvature = {k.x(), k.y(), k.z()};
            ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
            Near(p.output, expected); Near(p.output, independent, kDerivative);
            EXPECT_LT((Sum(p.output) + chrono::ChStarMatrix33<>(k)).norm(), 3e-14);
        }
    }
}

TEST_F(ReissnerFrameCuda, MeanRejectsInvalidUnitInputsAndPairwiseChartWithoutPublishingAndAllowsRetry) {
    const auto nodes = Noncoaxial();
    Packet p; Bind(p, nodes); ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
    const auto sentinel = p.mean;
    for (unsigned variant = 0; variant < 7; ++variant) {
        auto changed = nodes;
        auto expected = gpu::Status::kNonUnitQuaternion;
        if (variant == 0) changed[3] = {0, 0, 0, 0};
        if (variant == 1) changed[2] *= 1.00001;
        if (variant == 2) changed[1] = {1e300, 0, 0, 0};
        if (variant == 3) { changed[2].e1() = std::numeric_limits<double>::quiet_NaN(); expected = gpu::Status::kNonfiniteInput; }
        if (variant == 4) { changed[2].e2() = std::numeric_limits<double>::infinity(); expected = gpu::Status::kNonfiniteInput; }
        if (variant == 5) { changed[3] = Rotation({2, 0, 0}); expected = gpu::Status::kOutsideChart; }
        if (variant == 6) {
            changed = {{Rotation({0, 0, 0}), Rotation({1, 0, 0}), Rotation({-1, 0, 0}), Rotation({0, 0, 0})}};
            expected = gpu::Status::kOutsideChart;  // Both nonzero nodes pass the anchor-only chart.
        }
        Bind(p, changed); ASSERT_TRUE(Run(p)); EXPECT_EQ(p.status, expected); Unchanged(p.mean, sentinel);
        Bind(p, nodes); ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess); Unchanged(p.mean, sentinel);
    }
}

Packet CorrectionInput(Operation operation) {
    Packet p;
    p.operation = operation;
    const auto mean = CpuMean(Noncoaxial());
    p.spin = ToGpu(mean.spin);
    p.frozen = ToGpu(Spin{{Mat(.1), Mat(.2), Mat(.3), Mat(.15)}});
    p.curvature = {.1, .2, -.3};
    p.output = ToGpu(Spin{{Mat(11), Mat(13), Mat(17), Mat(19)}});
    return p;
}

TEST_F(ReissnerFrameCuda, CorrectionsRejectNonfiniteAndFiniteOverflowPreserveOutputAndRetry) {
    for (auto operation : {Operation::Orientation, Operation::Curvature}) {
        const auto valid = CorrectionInput(operation);
        for (unsigned variant = 0; variant < 5; ++variant) {
            if (variant == 4 && operation != Operation::Curvature) continue;
            auto p = valid;
            auto expected = gpu::Status::kNonfiniteInput;
            if (variant == 0) p.frozen.node[3].v[8] = std::numeric_limits<double>::quiet_NaN();
            if (variant == 1) p.spin.node[3].v[8] = std::numeric_limits<double>::infinity();
            if (variant == 2) {
                p.frozen = ToGpu(Spin{{Mat(1e308), Mat(1e308), Mat(1e308), Mat(1e308)}});
                expected = gpu::Status::kNonfiniteResult;  // Finite-input closure sum overflow.
            }
            if (variant == 3) {
                p.frozen = ToGpu(Spin{{Mat(-.5875), Mat(-.5875), Mat(-.5875), Mat(-.5875)}});
                for (auto& value : p.spin.node[3].v) value = 1e308;
                expected = gpu::Status::kNonfiniteResult;  // Late fourth-node matrix product overflow.
            }
            if (variant == 4) p.curvature.z = std::numeric_limits<double>::infinity();
            ASSERT_TRUE(Run(p)); EXPECT_EQ(p.status, expected); Unchanged(p.output, valid.output);
            p = valid; ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
        }
    }
}

TEST_F(ReissnerFrameCuda, BothCorrectionInputsCanAliasOutputWithTransactionalFailureAndReuse) {
    for (auto operation : {Operation::Orientation, Operation::Curvature}) {
        auto baseline = CorrectionInput(operation);
        ASSERT_TRUE(Run(baseline)); ASSERT_EQ(baseline.status, gpu::Status::kSuccess);
        for (unsigned alias : {1u, 2u}) {
            auto p = CorrectionInput(operation); p.alias = alias;
            ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
            Unchanged(alias == 1 ? p.frozen : p.spin, baseline.output);
            p = CorrectionInput(operation); p.alias = alias;
            p.frozen.node[3].v[8] = std::numeric_limits<double>::quiet_NaN();
            const auto before = alias == 1 ? p.frozen : p.spin;
            ASSERT_TRUE(Run(p)); EXPECT_EQ(p.status, gpu::Status::kNonfiniteInput);
            Unchanged(alias == 1 ? p.frozen : p.spin, before);
            p = CorrectionInput(operation); p.alias = alias;
            ASSERT_TRUE(Run(p)); ASSERT_EQ(p.status, gpu::Status::kSuccess);
            Unchanged(alias == 1 ? p.frozen : p.spin, baseline.output);
        }
    }
}
}  // namespace
