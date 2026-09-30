// Rotation primitives only. The real Chrono donor supplies direct comparisons;
// quaternion rotation values and a separate stable Jacobian formula supply
// derivative checks. No shell state, force assembly or dynamics are tested here.
#include "lib_src/elements/ReissnerRotation.h"
#include "chrono/core/ChMatrixMBD.h"
#include "chrono/fea/ChRotUtils.h"

#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {
namespace tlr = tl::fea::reissner;
namespace donor = chrono::fea::rotutils;
using Vec = chrono::ChVector3d;
using Mat = chrono::ChMatrix33<>;
using Quat = chrono::ChQuaterniond;

constexpr double kDirect = 1e-13;
constexpr double kDerivative = 2e-8;
constexpr double kPi = 3.14159265358979323846;

struct Packet {
    tlr::Vec3 phi{}, direction{.3, -.5, .7}, logarithm{11, 12, 13};
    tlr::Matrix3 input{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    tlr::Matrix3 rotation{}, jacobian{}, inverse{}, variation{};
    tlr::Status status[4]{};
};
static_assert(sizeof(Packet) < 1024, "Rotation probes must remain tiny");

__host__ __device__ void Evaluate(Packet& p) {
    p.status[0] = tlr::ComputeRotationAndJacobian(p.phi, p.rotation, p.jacobian);
    p.status[1] = tlr::ComputeRotationJacobianInverse(p.phi, p.inverse);
    p.status[2] = tlr::ComputeRotationJacobianVariation(p.phi, p.direction, p.variation);
    p.status[3] = tlr::ComputeRotationVector(p.input, p.logarithm);
}
__global__ void EvaluateDevice(Packet* p) {
    if (!blockIdx.x && !threadIdx.x) Evaluate(*p);
}
class ReissnerRotationCuda : public ::testing::Test {
  protected:
    Packet* device_ = nullptr;
    void SetUp() override {
        int count = 0;
        ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess);
        ASSERT_GT(count, 0);
        ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device_), sizeof(Packet)), cudaSuccess);
    }
    void TearDown() override {
        if (device_) EXPECT_EQ(cudaFree(device_), cudaSuccess);
    }
    ::testing::AssertionResult Run(Packet& p) {
        auto error = cudaMemcpy(device_, &p, sizeof(Packet), cudaMemcpyHostToDevice);
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        EvaluateDevice<<<1, 1>>>(device_);
        error = cudaGetLastError();
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        error = cudaDeviceSynchronize();
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        error = cudaMemcpy(&p, device_, sizeof(Packet), cudaMemcpyDeviceToHost);
        if (error != cudaSuccess) return ::testing::AssertionFailure() << cudaGetErrorString(error);
        return ::testing::AssertionSuccess();
    }
};

tlr::Vec3 ToTl(const Vec& a) { return {a.x(), a.y(), a.z()}; }
Vec ToCpu(tlr::Vec3 a) { return {a.x, a.y, a.z}; }
tlr::Matrix3 ToTl(const Mat& a) {
    tlr::Matrix3 result;
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col) result.v[3 * row + col] = a(row, col);
    return result;
}
Mat ToCpu(const tlr::Matrix3& a) {
    Mat result(0);
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col) result(row, col) = a.v[3 * row + col];
    return result;
}
Mat QuaternionRotation(const Vec& phi) {
    Quat q;
    q.SetFromRotVec(phi);
    return Mat(q);
}
Packet MakePacket(const Vec& phi) {
    Packet packet;
    packet.phi = ToTl(phi);
    packet.input = ToTl(QuaternionRotation(phi));
    return packet;
}
std::vector<Vec> BranchSamples() {
    std::vector<double> angles{0, 1e-12, 1e-6, .02, .3, 2.4, kPi - 1e-6};
    // Include the donor's distinct A/B/C/D/E switches, including both sides.
    for (double threshold : {1.1, 1.3, 1.5, 1.6, 1.7})
        for (double delta : {-1e-9, 0., 1e-9}) angles.push_back(threshold + delta);
    std::vector<Vec> samples;
    const Vec axis = Vec(.3, -.4, .5).GetNormalized();
    for (double angle : angles) {
        samples.push_back(axis * angle);
        samples.push_back(axis * -angle);
    }
    return samples;
}
void Near(const tlr::Matrix3& actual, const Mat& expected, double tolerance = kDirect) {
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col)
            EXPECT_NEAR(actual.v[3 * row + col], expected(row, col), tolerance) << row << ',' << col;
}
void CheckDonor(const Packet& p) {
    for (auto status : p.status) ASSERT_EQ(status, tlr::Status::kSuccess);
    Mat rotation, jacobian;
    donor::RotAndDRot(ToCpu(p.phi), rotation, jacobian);
    Near(p.rotation, rotation);
    Near(p.jacobian, jacobian);
    Near(p.inverse, donor::DRot_I(ToCpu(p.phi)));
    Near(p.variation, donor::Elle(ToCpu(p.phi), ToCpu(p.direction)));
    const auto logarithm = donor::VecRot(ToCpu(p.input));
    EXPECT_NEAR(p.logarithm.x, logarithm.x(), kDirect);
    EXPECT_NEAR(p.logarithm.y, logarithm.y(), kDirect);
    EXPECT_NEAR(p.logarithm.z, logarithm.z(), kDirect);
}
void Unchanged(const tlr::Matrix3& a, const tlr::Matrix3& b) {
    for (unsigned i = 0; i < 9; ++i)
        EXPECT_EQ(std::memcmp(&a.v[i], &b.v[i], sizeof(double)), 0) << i;
}
void OutputsUnchanged(const Packet& a, const Packet& b) {
    Unchanged(a.rotation, b.rotation);
    Unchanged(a.jacobian, b.jacobian);
    Unchanged(a.inverse, b.inverse);
    Unchanged(a.variation, b.variation);
    EXPECT_EQ(std::memcmp(&a.logarithm.x, &b.logarithm.x, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&a.logarithm.y, &b.logarithm.y, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&a.logarithm.z, &b.logarithm.z, sizeof(double)), 0);
}
void SeedOutputs(Packet& p) {
    for (unsigned i = 0; i < 9; ++i) {
        p.rotation.v[i] = 10 + i;
        p.jacobian.v[i] = 20 + i;
        p.inverse.v[i] = 30 + i;
        p.variation.v[i] = 40 + i;
    }
}

// Independent stable left-Jacobian value, using a half-angle expression and
// a short near-zero series rather than donor coefficient tables/thresholds.
Mat ValueJacobian(const Vec& phi) {
    const double angle = phi.Length(), square = angle * angle;
    const double b = angle < 1e-3 ? .5 - square / 24 + square * square / 720
                                  : 2 * std::pow(std::sin(angle / 2) / angle, 2);
    const double c = angle < 1e-3 ? 1. / 6 - square / 120 + square * square / 5040
                                  : (1 - std::sin(angle) / angle) / square;
    const Mat skew = chrono::ChStarMatrix33d(phi);
    return Mat(1) + b * skew + c * skew * skew;
}

TEST(ReissnerRotationHost, MatchesChronoAcrossSeriesAndLogBranches) {
    for (const auto& phi : BranchSamples()) {
        SCOPED_TRACE(phi.Length());
        auto p = MakePacket(phi);
        Evaluate(p);
        CheckDonor(p);
    }
}

TEST_F(ReissnerRotationCuda, MatchesChronoAcrossSeriesAndLogBranches) {
    for (const auto& phi : BranchSamples()) {
        SCOPED_TRACE(phi.Length());
        auto p = MakePacket(phi);
        ASSERT_TRUE(Run(p));
        CheckDonor(p);
    }
}

TEST_F(ReissnerRotationCuda, PreservesProperRotationInverseAndPrincipalRoundTrip) {
    for (const auto& phi : BranchSamples()) {
        auto p = MakePacket(phi);
        ASSERT_TRUE(Run(p));
        for (auto status : p.status) ASSERT_EQ(status, tlr::Status::kSuccess);
        const auto r = ToCpu(p.rotation), j = ToCpu(p.jacobian), inverse = ToCpu(p.inverse);
        EXPECT_LT((r.transpose() * r - Mat(1)).cwiseAbs().maxCoeff(), 3e-13);
        EXPECT_NEAR(r.determinant(), 1, 3e-13);
        EXPECT_LT((inverse * j - Mat(1)).cwiseAbs().maxCoeff(), 3e-13);
        EXPECT_LT((j * inverse - Mat(1)).cwiseAbs().maxCoeff(), 3e-13);
        EXPECT_LT((QuaternionRotation(ToCpu(p.logarithm)) - ToCpu(p.input)).cwiseAbs().maxCoeff(), 3e-13);
    }
}

TEST_F(ReissnerRotationCuda, RotationJacobianAndElleMatchIndependentDerivatives) {
    for (const Vec phi : {Vec(0, 0, 0), Vec(.13, -.21, .34), Vec(1.21, -.72, .83)}) {
        auto p = MakePacket(phi);
        ASSERT_TRUE(Run(p));
        ASSERT_EQ(p.status[0], tlr::Status::kSuccess);
        ASSERT_EQ(p.status[2], tlr::Status::kSuccess);
        const double delta = 2e-5;
        Mat derivative_j(0), derivative_l(0);
        for (unsigned axis = 0; axis < 3; ++axis) {
            Vec step(0, 0, 0);
            step[axis] = delta;
            const Mat dr = (QuaternionRotation(phi + step) - QuaternionRotation(phi - step)) / (2 * delta);
            const Mat spatial = dr * QuaternionRotation(phi).transpose();
            derivative_j(0, axis) = .5 * (spatial(2, 1) - spatial(1, 2));
            derivative_j(1, axis) = .5 * (spatial(0, 2) - spatial(2, 0));
            derivative_j(2, axis) = .5 * (spatial(1, 0) - spatial(0, 1));
            const Vec direction = ToCpu(p.direction);
            const Vec variation = (ValueJacobian(phi + step) * direction - ValueJacobian(phi - step) * direction) / (2 * delta);
            for (unsigned row = 0; row < 3; ++row) derivative_l(row, axis) = variation[row];
        }
        Near(p.jacobian, derivative_j, kDerivative);
        Near(p.variation, derivative_l, kDerivative);
    }
}

TEST_F(ReissnerRotationCuda, ElleFiniteDifferenceRefinesBeforeRoundoff) {
    const Vec phi(.23, -.37, .41);
    auto p = MakePacket(phi);
    ASSERT_TRUE(Run(p));
    ASSERT_EQ(p.status[2], tlr::Status::kSuccess);
    double previous = 1;
    for (double delta : {2e-3, 1e-3, 5e-4}) {
        Mat finite_difference(0);
        for (unsigned axis = 0; axis < 3; ++axis) {
            Vec step(0, 0, 0);
            step[axis] = delta;
            const Vec value = (ValueJacobian(phi + step) * ToCpu(p.direction) -
                               ValueJacobian(phi - step) * ToCpu(p.direction)) / (2 * delta);
            for (unsigned row = 0; row < 3; ++row) finite_difference(row, axis) = value[row];
        }
        const double error = (finite_difference - ToCpu(p.variation)).cwiseAbs().maxCoeff();
        EXPECT_LT(error, previous * .3);
        previous = error;
    }
    EXPECT_LT(previous, kDerivative);
}

TEST(ReissnerRotationHost, RejectsInvalidChartsAndKeepsOutputs) {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), kPi, 1e200}) {
        Packet p;
        SeedOutputs(p);
        p.phi.x = invalid;
        p.input.v[0] = invalid;
        const auto before = p;
        Evaluate(p);
        for (auto status : p.status) EXPECT_NE(status, tlr::Status::kSuccess);
        OutputsUnchanged(p, before);
    }
    for (const Mat invalid : {Mat(0), Mat(-1), Mat(2)}) {
        tlr::Vec3 output{7, 8, 9};
        EXPECT_EQ(tlr::ComputeRotationVector(ToTl(invalid), output), tlr::Status::kOutsideChart);
        EXPECT_DOUBLE_EQ(output.x, 7);
        EXPECT_DOUBLE_EQ(output.y, 8);
        EXPECT_DOUBLE_EQ(output.z, 9);
    }
    const tlr::Matrix3 half_turn{{1, 0, 0, 0, -1, 0, 0, 0, -1}};
    tlr::Vec3 output{7, 8, 9};
    EXPECT_EQ(tlr::ComputeRotationVector(half_turn, output), tlr::Status::kOutsideChart);
    EXPECT_DOUBLE_EQ(output.x, 7);
}

TEST_F(ReissnerRotationCuda, FailedOperationPreservesOutputsAndAllowsRetry) {
    Packet p;
    SeedOutputs(p);
    p.phi.x = std::numeric_limits<double>::quiet_NaN();
    p.input.v[0] = std::numeric_limits<double>::infinity();
    const auto before = p;
    ASSERT_TRUE(Run(p));
    for (auto status : p.status) EXPECT_EQ(status, tlr::Status::kNonfiniteInput);
    OutputsUnchanged(p, before);
    p.phi = {.13, -.27, .19};
    p.input = ToTl(QuaternionRotation(ToCpu(p.phi)));
    ASSERT_TRUE(Run(p));
    CheckDonor(p);

    const double large = std::numeric_limits<double>::max();
    p.phi = {1.3, -1.7, 1.4};
    p.direction = {large, large, large};
    const auto previous_variation = p.variation;
    ASSERT_TRUE(Run(p));
    EXPECT_EQ(p.status[2], tlr::Status::kNonfiniteResult);
    Unchanged(p.variation, previous_variation);
}

}  // namespace
