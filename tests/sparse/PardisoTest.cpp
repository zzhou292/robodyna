#include "Integration.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include <complex>
#include <type_traits>

static_assert(std::is_same_v<MKL_INT, int> && sizeof(MKL_INT) == 4);

TEST(PardisoSdk, RealFactorizationRetainsLp64AndPatternReuse) {
    chrono::ChSparseMatrix matrix(3, 3);
    matrix.coeffRef(0, 0) = 4;
    matrix.coeffRef(0, 1) = 1;
    matrix.coeffRef(1, 0) = 2;
    matrix.coeffRef(1, 1) = 5;
    matrix.coeffRef(1, 2) = 1;
    matrix.coeffRef(2, 1) = 3;
    matrix.coeffRef(2, 2) = 6;
    matrix.makeCompressed();
    chrono::ChVectorDynamic<> expected(3);
    expected << 1, -2, .5;
    chrono::ChSolverPardisoMKL solver(1);
    auto& engine = solver.GetMklEngine();
    engine.compute(matrix);
    ASSERT_EQ(engine.info(), Eigen::Success);
    const chrono::ChVectorDynamic<> rhs = matrix * expected;
    const chrono::ChVectorDynamic<> solution = engine.solve(rhs);
    ASSERT_EQ(engine.info(), Eigen::Success);
    EXPECT_LT((solution - expected).norm(), 1e-12);
    matrix.coeffRef(1, 1) = 7;
    engine.factorize(matrix);
    ASSERT_EQ(engine.info(), Eigen::Success);
    const chrono::ChVectorDynamic<> next_rhs = matrix * expected;
    const chrono::ChVectorDynamic<> next = engine.solve(next_rhs);
    EXPECT_LT((next - expected).norm(), 1e-12);
}

TEST(PardisoSdk, RealComplexFactorization) {
    using C = std::complex<double>;
    chrono::ChComplexSparseMatrix matrix(2, 2);
    matrix.coeffRef(0, 0) = C(3, 1);
    matrix.coeffRef(0, 1) = C(1, -.5);
    matrix.coeffRef(1, 0) = C(-2, 1);
    matrix.coeffRef(1, 1) = C(5, -1);
    matrix.makeCompressed();
    chrono::ChVectorDynamic<C> expected(2);
    expected << C(1, -2), C(.5, 1);
    chrono::ChSolverComplexPardisoMKL solver(1);
    auto& engine = solver.GetMklEngine();
    engine.compute(matrix);
    ASSERT_EQ(engine.info(), Eigen::Success);
    const chrono::ChVectorDynamic<C> rhs = matrix * expected;
    const chrono::ChVectorDynamic<C> result = engine.solve(rhs);
    ASSERT_EQ(engine.info(), Eigen::Success);
    EXPECT_LT((result - expected).norm(), 1e-12);
}

TEST(PardisoSdk, RealCoupledMechanicalSolveUsesPardiso) {
    ASSERT_NO_FATAL_FAILURE(robodyna::tests::sparse::CheckCoupledSolver(
        std::make_shared<chrono::ChSolverPardisoMKL>(1), chrono::ChSolver::Type::PARDISO_MKL));
}
