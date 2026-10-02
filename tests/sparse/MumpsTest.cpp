#include "Integration.h"
#include "chrono_mumps/ChMumpsEngine.h"
#include "chrono_mumps/ChSolverMumps.h"

#include <type_traits>

static_assert(std::is_same_v<MUMPS_INT, int> && sizeof(MUMPS_INT) == 4);

TEST(MumpsSdk, RealSequentialFactorizationAndChangedRightHandSide) {
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
    chrono::ChVectorDynamic<> rhs = matrix * expected;
    chrono::ChMumpsEngine engine;
    engine.SetNumThreads(1);
    engine.SetProblem(matrix, rhs);
    ASSERT_EQ(engine.MumpsCall(chrono::ChMumpsEngine::COMPLETE), 0);
    EXPECT_LT((rhs - expected).norm(), 1e-12);
    expected << -3, .25, 2;
    rhs = matrix * expected;
    engine.SetRhsVector(rhs);
    ASSERT_EQ(engine.MumpsCall(chrono::ChMumpsEngine::SOLVE), 0);
    EXPECT_LT((rhs - expected).norm(), 1e-12);
}

TEST(MumpsSdk, RealCoupledMechanicalSolveUsesMumps) {
    ASSERT_NO_FATAL_FAILURE(robodyna::tests::sparse::CheckCoupledSolver(
        std::make_shared<chrono::ChSolverMumps>(1), chrono::ChSolver::Type::MUMPS));
}
