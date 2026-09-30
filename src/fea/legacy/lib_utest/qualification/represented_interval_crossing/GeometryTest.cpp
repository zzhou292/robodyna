// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace represented_interval_test {

TEST(RepresentedIntervalCrossing,
     PassThroughWithSeparatedEndpointsCertifiesMiddleContact) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  auto upper = BaseTriangle(1);
  auto lower = BaseTriangle(-3);
  EXPECT_EQ(One(owner, {a, Static(20, upper)}).classification,
            C::CertifiedSeparated);
  EXPECT_EQ(One(owner, {a, Static(20, lower)}).classification,
            C::CertifiedSeparated);
  const auto b = Path(20, upper, lower);
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.reason, R::None);
  EXPECT_EQ(result.geometry, G::Coplanar);
  EXPECT_EQ(result.witness_time_numerator, 1u);
  EXPECT_EQ(result.witness_time_depth, 2u);
  EXPECT_EQ(result.work, 2u);
}

TEST(RepresentedIntervalCrossing,
     EnterAndExitBetweenEndpointsCertifiesTransverseIntersection) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{-2, -.5, -1}, {-2, 2.5, -1}, {-2, 1, 1}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{4, -.5, -1}, {4, 2.5, -1}, {4, 1, 1}};
  EXPECT_EQ(One(owner, {a, Static(20, first)}).classification,
            C::CertifiedSeparated);
  EXPECT_EQ(One(owner, {a, Static(20, second)}).classification,
            C::CertifiedSeparated);
  const auto result = One(owner, {a, Path(20, first, second)});
  EXPECT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.geometry, G::Transverse);
  EXPECT_EQ(result.feature.kind, K::TriangleIntersection);
  EXPECT_EQ(result.witness_time_numerator, 1u);
  EXPECT_EQ(result.witness_time_depth, 1u);
}

TEST(RepresentedIntervalCrossing,
     NearGrazingRepresentableGapIsCertifiedSeparated) {
  auto owner = Owner();
  const double boundary = std::nextafter(2.0, 3.0);
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{-3, boundary, 0}, {-2, boundary, 0},
      {-3, boundary + 1, 0}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{3, boundary, 0}, {4, boundary, 0},
      {3, boundary + 1, 0}};
  const auto result = One(
      owner, {Static(10, BaseTriangle()), Path(20, first, second)});
  EXPECT_EQ(result.classification, C::CertifiedSeparated);
  EXPECT_EQ(result.reason, R::None);
  EXPECT_EQ(result.work, 1u);
}

TEST(RepresentedIntervalCrossing,
     VertexFaceWitnessUsesImmutableVertexAndFaceKeys) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{.25, .25, 1}, {2.5, 2, 1}, {3, 2, 1}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{.25, .25, -1}, {2.5, 2, 1}, {3, 2, 1}};
  const auto b = Path(20, first, second, 200);
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.feature.kind, K::VertexFace);
  EXPECT_EQ(result.feature.vertex.first, 200u);
  EXPECT_EQ(result.feature.face.parent_eid, 10u);
}

TEST(RepresentedIntervalCrossing,
     EdgeEdgeWitnessUsesSortedImmutableEdgeKeys) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle(), 100);
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{.5, -.5, 1}, {.5, 2, 1}, {.5, 2.5, 2}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{.5, -.5, -1}, {.5, 2, -1}, {.5, 2.5, 2}};
  const auto b = Path(20, first, second, 200);
  const auto result = One(owner, {a, b});
  EXPECT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.geometry, G::Transverse);
  EXPECT_EQ(result.feature.kind, K::EdgeEdge);
  EXPECT_LE(result.feature.edges[0].endpoints[0].first,
            result.feature.edges[1].endpoints[0].first);
}

TEST(RepresentedIntervalCrossing,
     TransverseTriangleIntersectionIsNotReducedToEndpointDistances) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{-2, -.5, -1}, {-2, 2.5, -1}, {-2, 1, 1}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{4, -.5, -1}, {4, 2.5, -1}, {4, 1, 1}};
  const auto result = One(owner, {a, Path(20, first, second)});
  EXPECT_EQ(result.geometry, G::Transverse);
  EXPECT_EQ(result.feature.kind, K::TriangleIntersection);
}

TEST(RepresentedIntervalCrossing,
     CoplanarPassThroughIsCertifiedAtInteriorDyadicTime) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  const std::array<ct::Vec3, 3> first{
      ct::Vec3{-4, 0, 0}, {-2, 0, 0}, {-4, 2, 0}};
  const std::array<ct::Vec3, 3> second{
      ct::Vec3{4, 0, 0}, {6, 0, 0}, {4, 2, 0}};
  const auto result = One(owner, {a, Path(20, first, second)});
  EXPECT_EQ(result.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(result.geometry, G::Coplanar);
  EXPECT_EQ(result.witness_time_numerator, 1u);
  EXPECT_EQ(result.witness_time_depth, 1u);
}

TEST(RepresentedIntervalCrossing,
     ExactBoundaryAndNextafterOnBothSidesRemainDistinct) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle());
  const auto contact = One(owner, {a, Static(20, BaseTriangle(0))});
  EXPECT_EQ(contact.classification, C::CertifiedCrossingContact);
  EXPECT_EQ(contact.geometry, G::ExactCommonTranslationCoplanar);
  EXPECT_EQ(ct::BaseIntersectionGeometry(contact.geometry), G::Coplanar);

  const double above = std::nextafter(0.0, 1.0);
  const auto positive =
      One(owner, {a, Static(20, BaseTriangle(above))});
  EXPECT_EQ(positive.classification, C::CertifiedSeparated);

  const double below = std::nextafter(0.0, -1.0);
  const auto negative =
      One(owner, {a, Static(20, BaseTriangle(below))});
  EXPECT_EQ(negative.classification, C::CertifiedSeparated);
}

}  // namespace represented_interval_test
