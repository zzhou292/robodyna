// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace represented_interval_test {
namespace {

void RebuildEdges(ct::RepresentedTrianglePath* path) {
  for (unsigned i = 0; i < 3; ++i)
    path->edge_keys[i] =
        Edge(path->vertices[i].key, path->vertices[(i + 1) % 3].key);
}

}  // namespace

TEST(RepresentedIntervalCrossing,
     PreflightRejectsPathCountThatWouldTruncateIndices) {
  if (SIZE_MAX > UINT32_MAX) {
    ct::RepresentedIntervalLimits limits;
    limits.max_paths = static_cast<std::size_t>(UINT32_MAX) + 1;
    EXPECT_EQ(ct::RepresentedIntervalCrossing::Preflight(limits).report.status,
              S::InvalidInput);
  }
}

TEST(RepresentedIntervalCrossing,
     CanonicalFixedFacetIdentityRejectsMalformedVertexAndEdges) {
  const auto valid_a = Static(10, BaseTriangle(), 100);
  const auto valid_b = Static(20, BaseTriangle(1), 200);
  const ct::RepresentedTrianglePair pair{0, 1};
  for (unsigned fault = 0; fault < 5; ++fault) {
    SCOPED_TRACE(fault);
    auto owner = Owner();
    std::vector<ct::RepresentedTrianglePath> paths{valid_a, valid_b};
    if (fault == 0)
      paths[1].vertices[2].key.source_instance_id++;
    if (fault == 1)
      std::swap(paths[1].edge_keys[0].endpoints[0],
                paths[1].edge_keys[0].endpoints[1]);
    if (fault == 2)
      paths[1].edge_keys[0].parent_eid = paths[1].key.parent_eid;
    if (fault == 3) {
      paths[1].edge_keys[0].parent_boundary = false;
      paths[1].edge_keys[0].parent_eid = paths[1].key.parent_eid + 1;
    }
    if (fault == 4) {
      auto& key = paths[1].vertices[1].key;
      key.kind = ct::FacetVertexKind::SourceEdge;
      key.first = 300;
      key.second = 400;
      key.numerator = 2;
      key.denominator = 4;  // Not reduced.
      RebuildEdges(&paths[1]);
    }
    const auto report =
        owner.Certify(paths.data(), paths.size(), &pair, 1);
    EXPECT_EQ(report.status, S::InvalidInput);
    EXPECT_EQ(report.input_path, 1u);
    EXPECT_FALSE(owner.results().complete);
  }
}

TEST(RepresentedIntervalCrossing,
     SharedVertexLedgerRejectsDifferentTrajectoryOrMotion) {
  const ct::RepresentedTrianglePair pair{0, 1};
  for (unsigned fault = 0; fault < 2; ++fault) {
    auto owner = Owner();
    auto a = Static(10, BaseTriangle(), 100);
    auto b = Static(20, BaseTriangle(1), 200);
    b.vertices[0].key = a.vertices[0].key;
    if (fault == 1) {
      b.vertices[0].endpoint[0] = a.vertices[0].endpoint[0];
      b.vertices[0].endpoint[1] = a.vertices[0].endpoint[1];
      b.motion = ct::RepresentedMotion::RigidArc;
    }
    RebuildEdges(&b);
    const std::vector<ct::RepresentedTrianglePath> paths{a, b};
    const auto report =
        owner.Certify(paths.data(), paths.size(), &pair, 1);
    EXPECT_EQ(report.status, S::IdentityMismatch);
    EXPECT_EQ(report.input_path, 1u);
  }
}

TEST(RepresentedIntervalCrossing,
     CompatibleDuplicatePathAndEdgesCanonicalizeToOnePair) {
  auto owner = Owner();
  const auto a = Static(10, BaseTriangle(), 100);
  const auto b = Static(20, BaseTriangle(1), 200);
  const auto duplicate = Permute(a, {2, 1, 0});
  const std::vector<ct::RepresentedTrianglePath> paths{a, b, duplicate};
  const ct::RepresentedTrianglePair pairs[]{{0, 1}, {2, 1}};
  const auto report =
      owner.Certify(paths.data(), paths.size(), pairs, 2);
  ASSERT_EQ(report.status, S::Ok);
  EXPECT_EQ(report.unique_pairs, 1u);
  ASSERT_EQ(owner.results().count, 1u);
  EXPECT_EQ(owner.results().data[0].classification, C::CertifiedSeparated);
}

TEST(RepresentedIntervalCrossing,
     AllOwnedRangesRejectAliasesAndFailedCallsPreserveCurrentView) {
  auto owner = Owner();
  std::vector<ct::RepresentedTrianglePath> paths{
      Static(10, BaseTriangle()), Static(20, BaseTriangle(1))};
  const ct::RepresentedTrianglePair pair{0, 1};
  ASSERT_EQ(owner.Certify(paths.data(), paths.size(), &pair, 1).status,
            S::Ok);
  const auto first = owner.results();
  ASSERT_TRUE(first.complete);

  paths[1] = Static(20, BaseTriangle(2));
  ASSERT_EQ(owner.Certify(paths.data(), paths.size(), &pair, 1).status,
            S::Ok);
  const auto current = owner.results();
  ASSERT_TRUE(current.complete);
  ASSERT_NE(first.data, current.data);  // Successful publication expires it.
  const auto bytes = Bytes(current.data, current.count);

  auto report = owner.Certify(
      paths.data(), paths.size(),
      reinterpret_cast<const ct::RepresentedTrianglePair*>(first.data), 1);
  EXPECT_EQ(report.status, S::InvalidInput);  // Old view is staging storage.
  EXPECT_EQ(owner.results().data, current.data);
  EXPECT_EQ(Bytes(owner.results().data, owner.results().count), bytes);

  report = owner.Certify(
      reinterpret_cast<const ct::RepresentedTrianglePath*>(current.data), 1,
      nullptr, 0);
  EXPECT_EQ(report.status, S::InvalidInput);  // Current publication storage.
  EXPECT_EQ(owner.results().data, current.data);
  EXPECT_EQ(Bytes(owner.results().data, owner.results().count), bytes);
}

}  // namespace represented_interval_test
