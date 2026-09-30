// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "../contact_facets/Fixture.h"
#include "lib_src/collision/weighted_surface/Mapping.h"

#include <gtest/gtest.h>

namespace {
namespace ct = tlfea::contact;

TEST(FixedTriangleCurrentAdapter,
     EvaluatesImmutableFacetKeysAndCurrentComposedVertices) {
  facet_test::Fixture fixture;
  ct::FixedContactFacetBinding facets;
  ASSERT_EQ(facets.Initialize(fixture.surface, {{}, 1}).status,
            ct::FixedContactFacetStatus::Ok);
  const auto parent = fixture.Parent(100);
  ct::FixedContactFacet facet;
  ASSERT_EQ(facets.Describe(parent, 3, &facet).status,
            ct::FixedContactFacetStatus::Ok);
  const ct::Vec3 parent_positions[4]{{0, 0, 0}, {2, 0, 0},
                                      {2, 2, 1}, {0, 2, -0.5}};
  const auto positions = fixture.Positions(parent, parent_positions);

  ct::CurrentFixedTriangle current;
  ASSERT_EQ(ct::EvaluateCurrentFixedTriangle(
                facet, facet_test::View(positions), &current),
            ct::Status::kOk);
  EXPECT_EQ(current.key.source_instance_id, facet.source_instance_id);
  EXPECT_EQ(current.key.parent_eid, facet.source.source_parent_id);
  EXPECT_EQ(current.key.level, facet.level);
  EXPECT_EQ(current.key.local_facet, facet.local_facet);
  for (unsigned i = 0; i < 3; ++i) {
    ct::Vec3 expected;
    ASSERT_EQ(ct::EvaluateWeightedSurfacePosition(
                  facet_test::View(positions), facet.vertices[i], &expected),
              ct::Status::kOk);
    facet_test::Equal(current.vertices[i], expected);
    EXPECT_TRUE(ct::SameFacetVertexKey(current.vertex_keys[i],
                                      facet.vertex_keys[i]));
    EXPECT_TRUE(ct::SameFacetEdgeKey(current.edge_keys[i],
                                    facet.edge_keys[i]));
  }
}

TEST(FixedTriangleCurrentAdapter, FailurePreservesCompleteOutput) {
  ct::CurrentFixedTriangle output;
  output.key.parent_eid = 991;
  ct::FixedContactFacet facet;
  EXPECT_EQ(ct::EvaluateCurrentFixedTriangle(facet, {}, &output),
            ct::Status::kInvalidArgument);
  EXPECT_EQ(output.key.parent_eid, 991u);
  EXPECT_EQ(ct::EvaluateCurrentFixedTriangle(facet, {}, nullptr),
            ct::Status::kInvalidArgument);
}

}  // namespace
