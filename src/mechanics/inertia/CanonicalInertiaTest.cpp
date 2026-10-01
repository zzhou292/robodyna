#include "robodyna/mechanics/RbMassProperties.h"
#include "chrono/physics/ChMassProperties.h"
#include <gtest/gtest.h>
#include <memory>
#include <type_traits>

namespace mechanics = robodyna::mechanics;

// Canonical-first inclusion and a second translation unit exercise both routes.
static_assert(std::is_same_v<mechanics::RbMassProperties, chrono::ChMassProperties>);
static_assert(std::is_same_v<mechanics::RbInertiaUtils, chrono::ChInertiaUtils>);
static_assert(std::is_same_v<mechanics::CompositeInertia, chrono::CompositeInertia>);

TEST(CanonicalInertia, BothNamesResolveToOneFunctionAndObjectType) {
    using Rotate = chrono::ChMatrix33<> (*)(const chrono::ChMatrix33<>, const chrono::ChMatrix33<>);
    EXPECT_EQ(static_cast<Rotate>(&mechanics::RbInertiaUtils::RotateInertia),
              static_cast<Rotate>(&chrono::ChInertiaUtils::RotateInertia));
    auto canonical = std::make_shared<mechanics::CompositeInertia>();
    std::shared_ptr<chrono::CompositeInertia> legacy = canonical;
    EXPECT_EQ(canonical.get(), legacy.get());
}

TEST(CanonicalInertia, ClusterMassCenterAndTensorMatchAnalyticValues) {
    const std::vector<chrono::ChVector3d> positions{{0, 0, 0}, {2, 0, 0}};
    const std::vector<chrono::ChMatrix33<>> rotations(2, chrono::ChMatrix33<>(1));
    const std::vector<chrono::ChMatrix33<>> inertias(2, chrono::ChMatrix33<>(chrono::ChVector3d(2, 3, 4)));
    const std::vector<double> masses{1, 3};
    chrono::ChMatrix33<> result;
    chrono::ChVector3d center;
    double mass;
    mechanics::RbInertiaUtils::InertiaFromCluster(positions, rotations, inertias, masses, result, mass, center);
    EXPECT_DOUBLE_EQ(mass, 4);
    EXPECT_LT((center - chrono::ChVector3d(1.5, 0, 0)).Length(), 1e-14);
    EXPECT_LT((result - chrono::ChMatrix33<>(chrono::ChVector3d(4, 9, 11))).norm(), 1e-14);
}

TEST(CanonicalInertia, MaterialVoidRestoresMassCenterAndInertia) {
    mechanics::CompositeInertia result;
    const chrono::ChMatrix33<> base(chrono::ChVector3d(20, 30, 40));
    const chrono::ChMatrix33<> part(chrono::ChVector3d(2, 3, 4));
    const chrono::ChFrame<> offset(chrono::ChVector3d(1, 0, 0));
    result.AddComponent(chrono::ChFrame<>(), 10, base);
    result.AddComponent(offset, 2, part);
    EXPECT_DOUBLE_EQ(result.GetMass(), 12);
    EXPECT_NEAR(result.GetCOM().x(), 1.0 / 6, 1e-14);
    result.AddComponent(offset, 2, part, true);
    EXPECT_DOUBLE_EQ(result.GetMass(), 10);
    EXPECT_LT(result.GetCOM().Length(), 1e-14);
    EXPECT_LT((result.GetInertia() - base).norm(), 1e-14);
    EXPECT_LT((result.GetInertiaReference() - base).norm(), 1e-14);
}
