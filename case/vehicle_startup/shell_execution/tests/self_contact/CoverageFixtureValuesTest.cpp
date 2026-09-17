#include "CoverageFixtureCapture.h"

#include <gtest/gtest.h>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {
std::array<contact::CurrentFixedTriangle, 2> Triangles() {
    std::array<contact::CurrentFixedTriangle, 2> result;
    for (unsigned side = 0; side < 2; ++side) {
        result[side].key.source_instance_id = 7;
        result[side].key.parent_eid = 100 + side;
        for (unsigned vertex = 0; vertex < 3; ++vertex)
            result[side].vertex_keys[vertex].first = 1 + 3 * side + vertex;
        for (unsigned edge = 0; edge < 3; ++edge) {
            result[side].edge_keys[edge].endpoints[0] = result[side].vertex_keys[edge];
            result[side].edge_keys[edge].endpoints[1] = result[side].vertex_keys[(edge + 1) % 3];
        }
    }
    return result;
}
}  // namespace

TEST(VehicleSelfContactFixtureCapture, BoundaryVertexAndEdgeOwnersAreNotLost) {
    const auto triangles = Triangles();
    sct::AcceptedEventCertificate owner;
    owner.event.source_order = 931;
    auto& vf = owner.discovery.key.vertex_face;
    vf.vertex = triangles[0].vertex_keys[0];
    vf.target.SetFace(triangles[1].key);
    EXPECT_TRUE(CouldOwnFacetPairConservatively(triangles, owner));
    vf.target.SetEdge(triangles[1].edge_keys[0]);
    EXPECT_TRUE(CouldOwnFacetPairConservatively(triangles, owner));
    EXPECT_FALSE(CouldOwnQuadraticPair(triangles, owner));
    vf.target.SetVertex(triangles[1].vertex_keys[0]);
    EXPECT_TRUE(CouldOwnFacetPairConservatively(triangles, owner));
    EXPECT_TRUE(CouldOwnFacetPairConservatively({triangles[1], triangles[0]}, owner));
    EXPECT_FALSE(CouldOwnQuadraticPair(triangles, owner));
    vf.target.SetVertex(triangles[0].vertex_keys[1]);
    EXPECT_FALSE(CouldOwnFacetPairConservatively(triangles, owner));
    EXPECT_EQ(owner.event.source_order, 931u);
}

TEST(VehicleSelfContactFixtureCapture, EdgeOwnersPreserveOppositeFacetIdentity) {
    const auto triangles = Triangles();
    sct::AcceptedEventCertificate owner;
    owner.discovery.key.SetEdgeEdge();
    owner.discovery.key.edge_edge.edges[0] = triangles[0].edge_keys[0];
    owner.discovery.key.edge_edge.edges[1] = triangles[1].edge_keys[1];
    EXPECT_TRUE(CouldOwnFacetPairConservatively(triangles, owner));
    EXPECT_TRUE(CouldOwnFacetPairConservatively({triangles[1], triangles[0]}, owner));
    owner.discovery.key.edge_edge.edges[1] = triangles[0].edge_keys[1];
    EXPECT_FALSE(CouldOwnFacetPairConservatively(triangles, owner));
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
