// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <limits>

namespace facet_test {
TEST(FixedFacetMapping, WarpedFacetUsesComposedWeightsAndPreservesForceMomentAndPower) {
  Fixture fixture;
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface).status, S::Ok);
  const auto parent = fixture.Parent(100);
  ct::FixedContactFacet facet;
  ASSERT_EQ(binding.Describe(parent, 0, &facet).status, S::Ok);
  const double lambda[]{.25, .25, .5};
  ct::WeightedSurfacePoint point;
  ASSERT_EQ(ct::ComposeFacetPoint(facet, lambda, fixture.source.domain.node_count(), &point), ct::Status::kOk);
  EXPECT_EQ(point.weights[0], .25);
  EXPECT_EQ(point.weights[1], .25);
  EXPECT_EQ(point.weights[2], .5);
  EXPECT_EQ(point.weights[3], 0);
  const ct::Vec3 positions[]{{0,0,0},{2,0,0},{2,2,1},{0,2,0}};
  const ct::Vec3 velocities[]{{1,2,3},{-2,3,1},{4,-1,2},{7,8,9}};
  auto x = fixture.Positions(parent, positions), v = fixture.Positions(parent, velocities);
  ct::WeightedPointKinematics motion;
  ASSERT_EQ(ct::EvaluateWeightedSurfacePoint(View(x), View(v), point, &motion), ct::Status::kOk);
  Equal(motion.position, {1.5, 1, .5});
  double curved[4];
  ASSERT_EQ(ct::EvaluateQ4Shape(-.5, 0, curved), ct::Status::kOk);
  EXPECT_EQ(curved[2], .375);
  EXPECT_NE(motion.position.z, curved[2]); // N(mean parameters) is a different surface.
  const ct::Vec3 force{4,-2,1};
  ct::WeightedNodalForces projected;
  ASSERT_EQ(ct::ProjectWeightedSurfaceForce(point, View(x).node_count, force, &projected), ct::Status::kOk);
  ct::Vec3 resultant, moment;
  double work = 0;
  for (unsigned i = 0; i < projected.count; ++i) {
    resultant = ct::Add(resultant, projected.forces[i]);
    moment = ct::Add(moment, Cross(positions[i], projected.forces[i]));
    work += ct::Dot(velocities[i], projected.forces[i]);
    Equal(projected.couples[i], {});
  }
  Equal(resultant, force);
  Equal(moment, Cross(motion.position, force));
  EXPECT_EQ(work, ct::Dot(motion.velocity, force));
  for (std::size_t p = 0; p < fixture.surface.parents().size(); ++p) {
    if (fixture.surface.parents()[p].arity != 3) continue;
    ASSERT_EQ(binding.Describe(p, 0, &facet).status, S::Ok);
    ASSERT_EQ(ct::ComposeFacetPoint(facet, lambda, View(x).node_count, &point), ct::Status::kOk);
    EXPECT_EQ(point.count, 3u);
    for (unsigned i = 0; i < 3; ++i) EXPECT_EQ(point.weights[i], lambda[i]);
  }
}

TEST(FixedFacetMapping, LateMalformedSupportsAndBarycentricsPreserveOutput) {
  Fixture fixture;
  ct::FixedContactFacetBinding binding;
  ASSERT_EQ(binding.Initialize(fixture.surface, {{}, 2}).status, S::Ok);
  ct::FixedContactFacet facet;
  ASSERT_EQ(binding.Describe(fixture.Parent(100), 1, &facet).status, S::Ok);
  const auto valid = facet;
  ct::WeightedSurfacePoint output;
  output.count = 19;
  const auto before = qbat_binding_test::Bytes(output);
  const double lambda[]{.25,.25,.5};
  for (unsigned fault = 0; fault < 4; ++fault) {
    facet = valid;
    if (fault == 0) facet.vertices[2].weights[3] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 1) facet.vertices[2].nodes[3] = UINT32_MAX;
    if (fault == 2) std::swap(facet.vertices[2].nodes[2], facet.vertices[2].nodes[3]);
    if (fault == 3) facet.vertices[2].count = 2;
    EXPECT_NE(ct::ComposeFacetPoint(facet, lambda, fixture.source.domain.node_count(), &output), ct::Status::kOk);
    EXPECT_EQ(qbat_binding_test::Bytes(output), before);
  }
  for (const auto& bad : {std::array<double,3>{-.1,.6,.5}, std::array<double,3>{.5,.5,.5}}) {
    EXPECT_NE(ct::ComposeFacetPoint(valid, bad.data(), fixture.source.domain.node_count(), &output), ct::Status::kOk);
    EXPECT_EQ(qbat_binding_test::Bytes(output), before);
  }
  EXPECT_EQ(ct::ComposeFacetPoint(valid, lambda, fixture.source.domain.node_count(), &output), ct::Status::kOk);
}
} // namespace facet_test
