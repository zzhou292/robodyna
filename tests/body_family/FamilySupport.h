// Small inherited-body coupons, shared by pre-rename and post-rename builds.
#pragma once

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "chrono/assets/ChVisualModel.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterialNSC.h"

namespace robodyna::body_family_test {
extern std::string box_file;
inline double Pi() { return std::acos(-1.0); }

inline void ExpectMassInertia(chrono::ChBody& body, double mass, const chrono::ChVector3d& diagonal) {
    EXPECT_NEAR(body.GetMass(), mass, 1e-9);
    EXPECT_LT((body.GetInertiaXX() - diagonal).Length(), 1e-8);
    EXPECT_LT(body.GetInertiaXY().Length(), 1e-8);
}

inline void ExpectAttachments(chrono::ChBody& body, unsigned shapes) {
    EXPECT_EQ(body.IsCollisionEnabled(), shapes != 0);
    EXPECT_EQ(body.GetVisualModel() ? body.GetVisualModel()->GetNumShapes() : 0, shapes);
    EXPECT_EQ(body.GetCollisionModel() ? body.GetCollisionModel()->GetNumShapes() : 0, shapes);
}

template <class T>
void ExpectIdentity(const std::shared_ptr<T>& body, const char* tag) {
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(chrono::ChClassFactory::GetClassTagName(typeid(*body)), tag);
    EXPECT_EQ(chrono::ChCastingMap::GetClassnameFromPtrTypeindex(typeid(T*)), tag);
    auto* expected = static_cast<chrono::ChBodyFrame*>(body.get());
    void* raw = body.get();
    EXPECT_EQ(chrono::ChCastingMap::Convert(tag, "ChBodyFrame", raw), expected);
    EXPECT_EQ(chrono::ChCastingMap::Convert(typeid(T*), typeid(chrono::ChBody*), raw),
              static_cast<chrono::ChBody*>(body.get()));
    auto shared = chrono::ChCastingMap::Convert(tag, "ChBodyFrame", std::static_pointer_cast<void>(body));
    ASSERT_EQ(shared.get(), expected);
    EXPECT_FALSE(body.owner_before(shared));
    EXPECT_FALSE(shared.owner_before(body));
}

inline std::vector<chrono::ChVector3d> BoxVertices() {
    return {{2,3,4},{4,3,4},{4,7,4},{2,7,4},{2,3,10},{4,3,10},{4,7,10},{2,7,10}};
}

inline std::shared_ptr<chrono::ChTriangleMeshConnected> BoxMesh() {
    auto mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = BoxVertices();
    mesh->GetIndicesVertices() = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,4,7},{0,7,3},
                                 {1,2,6},{1,6,5},{0,1,5},{0,5,4},{3,7,6},{3,6,2}};
    return mesh;
}

inline void ExpectAuxBox(chrono::ChBodyAuxRef& body) {
    EXPECT_NEAR(body.GetMass(), 144, 1e-8);
    const auto frame = body.GetFrameCOMToRef();
    EXPECT_LT((frame.GetPos() - chrono::ChVector3d(3,5,7)).Length(), 1e-8);
    EXPECT_LT(body.GetFrameRefToAbs().GetPos().Length(), 1e-8);
    // Compare inertia in the geometric reference frame, independent of the
    // eigenvector signs/order chosen for this body's principal COM axes.
    const chrono::ChMatrix33<> represented = frame.GetRotMat() * body.GetInertia() * frame.GetRotMat().transpose();
    const chrono::ChMatrix33<> expected(chrono::ChVector3d(624,480,240));
    EXPECT_LT((represented - expected).norm(), 1e-7);
}
}  // namespace robodyna::body_family_test
