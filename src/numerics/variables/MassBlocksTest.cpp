#include <gtest/gtest.h>
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/serialization/ChArchiveJSON.h"

#include <sstream>

TEST(NeutralMassBlocks, MassProductAndInverseAreConsistent) {
    chrono::ChVariablesBodyOwnMass block;
    block.SetBodyMass(3);
    block.SetBodyInertia(chrono::ChMatrix33<>(chrono::ChVector3d(2, 4, 5)));
    chrono::ChVectorDynamic<> velocity(6), momentum(6), recovered(6);
    velocity << 1, -2, .5, 3, -.25, 2;
    momentum.setZero();
    block.AddMassTimesVector(momentum, velocity);
    EXPECT_DOUBLE_EQ(momentum(0), 3);
    EXPECT_DOUBLE_EQ(momentum(1), -6);
    EXPECT_DOUBLE_EQ(momentum(2), 1.5);
    EXPECT_DOUBLE_EQ(momentum(3), 6);
    EXPECT_DOUBLE_EQ(momentum(4), -1);
    EXPECT_DOUBLE_EQ(momentum(5), 10);
    block.ComputeMassInverseTimesVector(recovered, momentum);
    EXPECT_LT((recovered - velocity).norm(), 1e-14);
}

TEST(NeutralMassBlocks, ArchiveRebuildsInverseMassAndInertia) {
    chrono::ChVariablesBodyOwnMass block;
    block.SetBodyMass(2.5);
    block.SetBodyInertia(chrono::ChMatrix33<>(chrono::ChVector3d(3, 4, 5)));
    std::stringstream stream;
    {
        chrono::ChArchiveOutJSON output(stream);
        output << CHNVP(block);
    }
    chrono::ChVariablesBodyOwnMass restored;
    {
        chrono::ChArchiveInJSON input(stream);
        input >> CHNVP(restored, "block");
    }
    chrono::ChVectorDynamic<> rhs(6), result(6);
    rhs << 2.5, 5, -2.5, 3, 8, -5;
    restored.ComputeMassInverseTimesVector(result, rhs);
    chrono::ChVectorDynamic<> expected(6);
    expected << 1, 2, -1, 1, 2, -1;
    EXPECT_LT((result - expected).norm(), 1e-14);
}
