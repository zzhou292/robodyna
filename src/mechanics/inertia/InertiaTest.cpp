#include <gtest/gtest.h>
#include "chrono/core/ChRotation.h"
#include "chrono/physics/ChMassProperties.h"
#include "robodyna/mechanics/RbMassProperties.h"
#include "chrono/utils/ChConstants.h"
#include <type_traits>

// Legacy-first inclusion must expose the canonical types, not replacement wrappers.
static_assert(std::is_same_v<chrono::ChMassProperties, robodyna::mechanics::RbMassProperties>);
static_assert(std::is_same_v<chrono::ChInertiaUtils, robodyna::mechanics::RbInertiaUtils>);
static_assert(std::is_same_v<chrono::CompositeInertia, robodyna::mechanics::CompositeInertia>);

TEST(NeutralInertia, ParallelAxisAndRotationMatchAnalyticValues) {
    const chrono::ChMatrix33<> diagonal(chrono::ChVector3d(2, 3, 4));
    const auto shifted = chrono::ChInertiaUtils::TranslateInertia(diagonal, {2, 0, 0}, 3);
    const chrono::ChMatrix33<> expected(chrono::ChVector3d(2, 15, 16));
    EXPECT_LT((shifted - expected).norm(), 1e-14);
    const chrono::ChMatrix33<> rotation(chrono::QuatFromAngleZ(chrono::CH_PI_2));
    const auto rotated = chrono::ChInertiaUtils::RotateInertia(diagonal, rotation);
    EXPECT_LT((rotated - chrono::ChMatrix33<>(chrono::ChVector3d(3, 2, 4))).norm(), 1e-13);
    EXPECT_LT((chrono::ChInertiaUtils::RotateInertia(rotated, rotation.transpose()) - diagonal).norm(), 1e-13);
}

TEST(NeutralInertia, PrincipalAxesReconstructRotatedTensor) {
    const chrono::ChMatrix33<> diagonal(chrono::ChVector3d(2, 3, 4));
    const chrono::ChMatrix33<> rotation(chrono::QuatFromAngleY(.37));
    const auto tensor = chrono::ChInertiaUtils::RotateInertia(diagonal, rotation);
    chrono::ChVector3d moments;
    chrono::ChMatrix33<> axes;
    chrono::ChInertiaUtils::PrincipalInertia(tensor, moments, axes);
    EXPECT_LT((moments - chrono::ChVector3d(2, 3, 4)).Length(), 1e-13);
    EXPECT_NEAR(axes.determinant(), 1, 1e-13);
    EXPECT_LT((axes * chrono::ChMatrix33<>(moments) * axes.transpose() - tensor).norm(), 1e-13);
}
