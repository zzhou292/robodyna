// SPDX-License-Identifier: MIT
#include "Fixture.h"

#include <gtest/gtest.h>

#include <array>
#include <set>

namespace {
namespace ft = fixed_triangle_test;
namespace ct = tlfea::contact;

struct MeshTriangle {
  ct::CurrentFixedTriangle value;
  std::array<std::uint64_t, 3> ids;
};

bool Contains(const std::array<std::uint64_t, 3>& ids,
              std::uint64_t value) {
  return std::find(ids.begin(), ids.end(), value) != ids.end();
}

bool IncidentEdges(const MeshTriangle& a, unsigned edge_a,
                   const MeshTriangle& b, unsigned edge_b) {
  const std::uint64_t aa[2]{a.ids[edge_a], a.ids[(edge_a + 1) % 3]};
  const std::uint64_t bb[2]{b.ids[edge_b], b.ids[(edge_b + 1) % 3]};
  return aa[0] == bb[0] || aa[0] == bb[1] ||
         aa[1] == bb[0] || aa[1] == bb[1];
}

void ExpectSameCandidate(
    const ct::FixedTriangleFeatureCandidate& a,
    const ct::FixedTriangleFeatureCandidate& b) {
  EXPECT_TRUE(ft::Same(a.key, b.key));
  for (unsigned i = 0; i < 2; ++i) {
    EXPECT_TRUE(ft::Same(a.triangles[i], b.triangles[i]));
    EXPECT_EQ(a.local_features[i], b.local_features[i]);
    EXPECT_EQ(a.points[i].x, b.points[i].x);
    EXPECT_EQ(a.points[i].y, b.points[i].y);
    EXPECT_EQ(a.points[i].z, b.points[i].z);
    EXPECT_EQ(a.edge_parameters[i], b.edge_parameters[i]);
  }
  for (unsigned i = 0; i < 3; ++i)
    EXPECT_EQ(a.face_weights[i], b.face_weights[i]);
  EXPECT_EQ(a.distance_m, b.distance_m);
}

std::size_t AddExpected(const MeshTriangle& input_a,
                        const MeshTriangle& input_b,
                        std::set<ft::Task>* expected) {
  const MeshTriangle* a = &input_a;
  const MeshTriangle* b = &input_b;
  if (a->value.key.parent_eid > b->value.key.parent_eid)
    std::swap(a, b);
  std::size_t count = 0;
  for (unsigned i = 0; i < 3; ++i) {
    if (!Contains(b->ids, a->ids[i])) {
      expected->insert({a->value.key.parent_eid, 0,
                        b->value.key.parent_eid, 0,
                        ct::FixedTriangleCandidateKind::VertexFace, i, 3});
      ++count;
    }
    if (!Contains(a->ids, b->ids[i])) {
      expected->insert({a->value.key.parent_eid, 0,
                        b->value.key.parent_eid, 0,
                        ct::FixedTriangleCandidateKind::VertexFace, 3, i});
      ++count;
    }
  }
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      if (!IncidentEdges(*a, i, *b, j)) {
        expected->insert({a->value.key.parent_eid, 0,
                          b->value.key.parent_eid, 0,
                          ct::FixedTriangleCandidateKind::EdgeEdge, i, j});
        ++count;
      }
  return count;
}

TEST(FixedTriangleExhaustive,
     EverySmallMeshTaskEqualsIndependentLocalIncidenceEnumeration) {
  const ct::Vec3 p0[3]{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  const ct::Vec3 p1[3]{{2, 0, 0}, {0, 0, 0}, {2, -2, 0}};
  const ct::Vec3 p2[3]{{0, 2, 0}, {-1, 3, 1}, {1, 3, -1}};
  const ct::Vec3 p3[3]{{0.25, 0.25, 0.5}, {1.25, 0.25, 0.5},
                       {0.25, 1.25, 0.5}};
  const std::uint64_t i0[3]{1, 2, 3};
  const std::uint64_t i1[3]{2, 1, 4};
  const std::uint64_t i2[3]{3, 5, 6};
  const std::uint64_t i3[3]{11, 12, 13};
  const MeshTriangle mesh[4]{
      {ft::Triangle(100, 0, p0, i0), {1, 2, 3}},
      {ft::Triangle(200, 0, p1, i1), {2, 1, 4}},
      {ft::Triangle(300, 0, p2, i2), {3, 5, 6}},
      {ft::Triangle(400, 0, p3, i3), {11, 12, 13}}};

  std::vector<ct::FixedTrianglePair> pairs;
  std::set<ft::Task> expected;
  std::size_t expected_raw = 0;
  for (unsigned i = 0; i < 4; ++i) {
    for (unsigned j = i + 1; j < 4; ++j) {
      pairs.push_back({i, j});
      pairs.push_back({j, i});
      expected_raw += 2 * AddExpected(mesh[i], mesh[j], &expected);
    }
  }
  ct::CurrentFixedTriangle values[4];
  for (unsigned i = 0; i < 4; ++i)
    values[i] = mesh[i].value;
  ct::FixedTriangleFeatureDiscovery discovery;
  ft::Initialize(&discovery, ft::Limits(pairs.size()));
  auto report = discovery.Discover(values, 4, pairs.data(), pairs.size());
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15 * pairs.size());
  EXPECT_EQ(report.raw_feature_candidates, expected_raw);
  const auto first_tasks = ft::Tasks(discovery.features());
  const std::vector<ct::FixedTriangleFeatureCandidate> first_values(
      discovery.features().data,
      discovery.features().data + discovery.features().count);
  const std::set<ft::Task> actual(first_tasks.begin(), first_tasks.end());
  for (const auto& task : actual)
    EXPECT_EQ(expected.count(task), 1u);
  EXPECT_LE(actual.size(), expected.size());

  // Permute both catalog and pair order.  Stable source keys, not ordinals,
  // define the publication.
  const unsigned permutation[4]{2, 0, 3, 1};
  unsigned inverse[4]{};
  for (unsigned i = 0; i < 4; ++i) {
    values[i] = mesh[permutation[i]].value;
    inverse[permutation[i]] = i;
  }
  for (auto& pair : pairs) {
    pair.first = inverse[pair.first];
    pair.second = inverse[pair.second];
  }
  std::reverse(pairs.begin(), pairs.end());
  report = discovery.Discover(values, 4, pairs.data(), pairs.size());
  ASSERT_EQ(report.status, ct::FixedTriangleDiscoveryStatus::Ok);
  EXPECT_EQ(report.feature_tasks, 15 * pairs.size());
  EXPECT_EQ(report.raw_feature_candidates, expected_raw);
  const auto tasks = ft::Tasks(discovery.features());
  EXPECT_EQ(tasks, first_tasks);
  ASSERT_EQ(discovery.features().count, first_values.size());
  for (std::size_t i = 0; i < first_values.size(); ++i)
    ExpectSameCandidate(discovery.features().data[i], first_values[i]);
}

}  // namespace
