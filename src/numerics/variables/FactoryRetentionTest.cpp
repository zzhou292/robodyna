#include <gtest/gtest.h>
#include "chrono/core/ChClassFactory.h"
#include "chrono/solver/ChVariables.h"

#include <memory>

// Deliberately no derived-class header, constructor or RTTI reference. Without
// retained registration the owning archive member would not be linked here.
TEST(NeutralMassBlocks, StaticFactoryRegistrationSurvivesIndependentLink) {
    ASSERT_TRUE(chrono::ChClassFactory::IsClassRegistered("ChVariablesBodyOwnMass"));
    chrono::ChVariables* raw = nullptr;
    ASSERT_NO_THROW(chrono::ChClassFactory::create("ChVariablesBodyOwnMass", &raw));
    const std::unique_ptr<chrono::ChVariables> block(raw);
    ASSERT_NE(block, nullptr);
    EXPECT_EQ(block->GetDOF(), 6u);
    chrono::ChVectorDynamic<> unit(6), result(6);
    unit.setOnes();
    block->ComputeMassInverseTimesVector(result, unit);
    EXPECT_LT((result - unit).norm(), 1e-14);
}
