// Canonical-first includes catch accidental reliance on legacy alias declarations.
#include "robodyna/mbd/RbBodyAuxRef.h"
#include "robodyna/mbd/RbBodyEasy.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChBodyEasy.h"

#include <gtest/gtest.h>
#include <type_traits>
#include <utility>

namespace robodyna::body_family_test {
static_assert(std::is_same_v<mbd::RbBodyAuxRef, chrono::ChBodyAuxRef>);
static_assert(std::is_same_v<mbd::RbBodyEasySphere, chrono::ChBodyEasySphere>);
static_assert(std::is_same_v<mbd::RbBodyEasyEllipsoid, chrono::ChBodyEasyEllipsoid>);
static_assert(std::is_same_v<mbd::RbBodyEasyCylinder, chrono::ChBodyEasyCylinder>);
static_assert(std::is_same_v<mbd::RbBodyEasyBox, chrono::ChBodyEasyBox>);
static_assert(std::is_same_v<mbd::RbBodyEasyConvexHull, chrono::ChBodyEasyConvexHull>);
static_assert(std::is_same_v<mbd::RbBodyEasyConvexHullAuxRef, chrono::ChBodyEasyConvexHullAuxRef>);
static_assert(std::is_same_v<mbd::RbBodyEasyMesh, chrono::ChBodyEasyMesh>);
static_assert(std::is_same_v<mbd::RbBodyEasyClusterOfSpheres, chrono::ChBodyEasyClusterOfSpheres>);
static_assert(std::is_same_v<decltype(std::declval<const mbd::RbBodyAuxRef&>().Clone()), mbd::RbBodyAuxRef*>);
// Easy classes retain inherited Clone behavior; this migration does not invent a derived clone.
static_assert(std::is_same_v<decltype(std::declval<const mbd::RbBodyEasyBox&>().Clone()), mbd::RbBody*>);
static_assert(std::is_same_v<decltype(std::declval<const mbd::RbBodyEasyMesh&>().Clone()), mbd::RbBodyAuxRef*>);

template <class T>
void ExpectRegistered(const char* legacy_name) {
    EXPECT_TRUE(chrono::ChClassFactory::IsClassRegistered(typeid(T)));
    EXPECT_EQ(chrono::ChClassFactory::GetClassTagName(typeid(T)), legacy_name);
}

TEST(BodyFamilyCanonicalApi, KeepsEveryLegacyFactoryIdentity) {
    // No constructor reference: the aggregate must retain its actual static registrars.
    ExpectRegistered<mbd::RbBodyAuxRef>("ChBodyAuxRef");
    ExpectRegistered<mbd::RbBodyEasySphere>("ChBodyEasySphere");
    ExpectRegistered<mbd::RbBodyEasyEllipsoid>("ChBodyEasyEllipsoid");
    ExpectRegistered<mbd::RbBodyEasyCylinder>("ChBodyEasyCylinder");
    ExpectRegistered<mbd::RbBodyEasyBox>("ChBodyEasyBox");
    ExpectRegistered<mbd::RbBodyEasyConvexHull>("ChBodyEasyConvexHull");
    ExpectRegistered<mbd::RbBodyEasyConvexHullAuxRef>("ChBodyEasyConvexHullAuxRef");
    ExpectRegistered<mbd::RbBodyEasyMesh>("ChBodyEasyMesh");
    ExpectRegistered<mbd::RbBodyEasyClusterOfSpheres>("ChBodyEasyClusterOfSpheres");
}
}  // namespace robodyna::body_family_test
