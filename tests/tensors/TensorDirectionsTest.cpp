#include "chrono/core/ChTensors.h"
#include "gtest/gtest.h"

#include <limits>
#include <type_traits>

namespace {

template <class Real, class Tensor>
void CheckRotatedTensor() {
    // Columns form a right-handed orthonormal basis; every axis is rotated.
    // Distinct mixed-sign eigenvalues avoid degenerate subspaces and exercise
    // all three shear components, including engineering-strain conversion.
    chrono::ChMatrix33<Real> rotation;
    rotation << Real(1), Real(2), Real(-2),
                Real(2), Real(1), Real(2),
                Real(2), Real(-2), Real(-1);
    rotation /= Real(3);
    const Real scale = std::is_same_v<Tensor, chrono::ChStressTensor<Real>> ? Real(1) : Real(0.01);
    chrono::ChVectorN<Real, 3> known;
    known << Real(-5) * scale, Real(2) * scale, Real(11) * scale;
    chrono::ChMatrix33<Real> matrix;
    matrix = rotation * known.asDiagonal() * rotation.transpose();

    Tensor tensor;
    tensor.ConvertFromMatrix(matrix);
    chrono::ChMatrix33<Real> restored;
    tensor.ConvertToMatrix(restored);
    const Real tolerance = Real(64) * std::numeric_limits<Real>::epsilon();
    EXPECT_LE((restored - matrix).norm(), tolerance * matrix.norm());

    double eigenvalue[3];
    chrono::ChVector3<Real> direction[3];
    if constexpr (std::is_same_v<Tensor, chrono::ChStressTensor<Real>>) {
        tensor.ComputePrincipalStressesDirections(eigenvalue[0], eigenvalue[1], eigenvalue[2],
                                                 direction[0], direction[1], direction[2]);
    } else {
        tensor.ComputePrincipalStrainsDirections(eigenvalue[0], eigenvalue[1], eigenvalue[2],
                                                direction[0], direction[1], direction[2]);
    }

    chrono::ChMatrix33<Real> basis;
    chrono::ChVectorN<Real, 3> values;
    for (int column = 0; column < 3; ++column) {
        basis(0, column) = direction[column].x();
        basis(1, column) = direction[column].y();
        basis(2, column) = direction[column].z();
        values(column) = Real(eigenvalue[column]);
        ASSERT_TRUE(basis.col(column).allFinite());
        EXPECT_NEAR(eigenvalue[column], double(known(column)), double(tolerance * matrix.norm()));
        EXPECT_LE((matrix * basis.col(column) - values(column) * basis.col(column)).norm(),
                  tolerance * matrix.norm());
    }
    // Eigenvector signs are arbitrary: validate the subspace, not a preferred sign.
    EXPECT_LE((basis.transpose() * basis - chrono::ChMatrix33<Real>::Identity()).norm(), tolerance);
    EXPECT_LE((basis * values.asDiagonal() * basis.transpose() - matrix).norm(), tolerance * matrix.norm());
}

TEST(TensorDirections, RotatedStressDouble) { CheckRotatedTensor<double, chrono::ChStressTensor<double>>(); }
TEST(TensorDirections, RotatedStressFloat) { CheckRotatedTensor<float, chrono::ChStressTensor<float>>(); }
TEST(TensorDirections, RotatedStrainDouble) { CheckRotatedTensor<double, chrono::ChStrainTensor<double>>(); }
TEST(TensorDirections, RotatedStrainFloat) { CheckRotatedTensor<float, chrono::ChStrainTensor<float>>(); }
TEST(TensorDirections, RotatedEngineeringStrainDouble) { CheckRotatedTensor<double, chrono::ChStrainEngTensor<double>>(); }
TEST(TensorDirections, RotatedEngineeringStrainFloat) { CheckRotatedTensor<float, chrono::ChStrainEngTensor<float>>(); }

}  // namespace
