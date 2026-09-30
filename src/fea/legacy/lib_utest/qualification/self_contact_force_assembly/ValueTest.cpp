// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/SelfContactForceValues.h"

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>

namespace self_contact_force_test {
namespace c = tlfea::contact;

c::WeightedSurfacePoint Point(std::array<std::uint32_t, 3> nodes) {
  c::WeightedSurfacePoint point;
  point.count = 3;
  for (unsigned i = 0; i < 3; ++i) {
    point.nodes[i] = nodes[i];
    point.weights[i] = i == 0 ? .5 : .25;
  }
  return point;
}

c::SelfContactForceEvent Event(std::uint64_t key,
                               std::uint64_t order,
                               std::array<std::uint32_t, 3> a,
                               std::array<std::uint32_t, 3> b) {
  c::SelfContactForceEvent event;
  event.source_order = order;
  event.feature.vertex_face.vertex.source_instance_id = 7;
  event.feature.vertex_face.vertex.first = key;
  c::FacetVertexKey target;
  target.source_instance_id = 7;
  target.first = 100 + key;
  event.feature.vertex_face.target.SetVertex(target);
  event.endpoints[0] = Point(a);
  event.endpoints[1] = Point(b);
  return event;
}

TEST(SelfContactForceValues, DirectedAreaDerivesOnlyOutwardRepresentedK) {
  const c::Q4CertifiedIntegral area{.25, .25, .25, 0};
  double stiffness = -1;
  ASSERT_TRUE(c::RepresentedSelfContactStiffness(12, area, &stiffness));
  EXPECT_EQ(stiffness, std::nextafter(3., HUGE_VAL));
  EXPECT_TRUE(c::PositiveSelfContactArea(area));
  auto zero = area;
  zero.lower = 0;
  EXPECT_FALSE(c::RepresentedSelfContactStiffness(12, zero, &stiffness));
  const c::Q4CertifiedIntegral huge{
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(), 0};
  EXPECT_FALSE(c::RepresentedSelfContactStiffness(4, huge, &stiffness));
  const c::Q4CertifiedIntegral tiny{
      std::numeric_limits<double>::denorm_min(),
      std::numeric_limits<double>::denorm_min(),
      std::numeric_limits<double>::denorm_min(), 0};
  EXPECT_FALSE(c::RepresentedSelfContactStiffness(.5, tiny, &stiffness));
}

TEST(SelfContactForceValues,
     CanonicalEventsAndPerNodeIncidentEventsHaveFixedOrder) {
  std::array<c::SelfContactForceEvent, 3> events{{
      Event(30, 8, {0, 1, 2}, {2, 3, 4}),
      Event(10, 9, {0, 4, 5}, {5, 6, 7}),
      Event(20, 7, {0, 2, 7}, {3, 6, 7})}};
  std::array<c::SelfContactForceIncidence, 24> incidence;
  std::array<c::SelfContactForceNodeIncidence, 8> nodes;
  c::SelfContactForceIncidenceSummary summary;
  const auto report = c::BuildSelfContactForceIncidence(
      events.data(), events.size(), 8, incidence.data(), incidence.size(),
      nodes.data(), nodes.size(), &summary);
  ASSERT_EQ(report.status, c::SelfContactForceStatus::Ok);
  EXPECT_EQ(events[0].feature.vertex_face.vertex.first, 10u);
  EXPECT_EQ(events[1].feature.vertex_face.vertex.first, 20u);
  EXPECT_EQ(events[2].feature.vertex_face.vertex.first, 30u);
  ASSERT_EQ(summary.touched_nodes, 8u);
  const auto zero = nodes[0];
  ASSERT_EQ(zero.node, 0u);
  ASSERT_EQ(zero.count, 3u);
  EXPECT_EQ(incidence[zero.offset + 0].event, 0u);
  EXPECT_EQ(incidence[zero.offset + 1].event, 1u);
  EXPECT_EQ(incidence[zero.offset + 2].event, 2u);
  for (std::size_t i = 1; i < summary.incidences; ++i)
    EXPECT_TRUE(incidence[i - 1].node < incidence[i].node ||
        (incidence[i - 1].node == incidence[i].node &&
         incidence[i - 1].event < incidence[i].event));
}

TEST(SelfContactForceValues,
     SameGeometryWithDistinctOrderedParentsHasDistinctIncidence) {
  std::array<c::SelfContactForceEvent, 2> events{{
      Event(10, 8, {0, 1, 2}, {2, 3, 4}),
      Event(10, 7, {0, 4, 5}, {5, 6, 7})}};
  events[0].classification.parent[0] = 4;
  events[0].classification.parent[1] = 5;
  events[1].classification.parent[0] = 2;
  events[1].classification.parent[1] = 3;
  std::array<c::SelfContactForceIncidence, 16> incidence;
  std::array<c::SelfContactForceNodeIncidence, 8> nodes;
  c::SelfContactForceIncidenceSummary summary;
  ASSERT_EQ(c::BuildSelfContactForceIncidence(
      events.data(), events.size(), 8, incidence.data(),
      incidence.size(), nodes.data(), nodes.size(), &summary).status,
      c::SelfContactForceStatus::Ok);
  EXPECT_EQ(events[0].classification.parent[0], 2u);
  EXPECT_EQ(events[1].classification.parent[0], 4u);
  EXPECT_GT(summary.incidences, 0u);

  events[1].classification.parent[0] = 2;
  events[1].classification.parent[1] = 3;
  EXPECT_EQ(c::BuildSelfContactForceIncidence(
      events.data(), events.size(), 8, incidence.data(),
      incidence.size(), nodes.data(), nodes.size(), &summary).status,
      c::SelfContactForceStatus::DuplicateEvent);
}

TEST(SelfContactForceValues, EmptyBatchPublishesExactEmptySummary) {
  c::SelfContactForceIncidenceSummary summary{71, 73};
  const auto report = c::BuildSelfContactForceIncidence(
      nullptr, 0, 8, nullptr, 0, nullptr, 0, &summary);
  EXPECT_EQ(report.status, c::SelfContactForceStatus::Ok);
  EXPECT_EQ(summary.incidences, 0u);
  EXPECT_EQ(summary.touched_nodes, 0u);
}

TEST(SelfContactForceValues,
     DuplicateCapsAndAliasesRejectWithoutSummaryPublication) {
  std::array<c::SelfContactForceEvent, 2> duplicate{{
      Event(10, 1, {0, 1, 2}, {2, 3, 4}),
      Event(10, 2, {0, 4, 5}, {5, 6, 7})}};
  std::array<c::SelfContactForceIncidence, 16> incidence;
  std::array<c::SelfContactForceNodeIncidence, 8> nodes;
  c::SelfContactForceIncidenceSummary summary{71, 73};
  EXPECT_EQ(c::BuildSelfContactForceIncidence(
      duplicate.data(), duplicate.size(), 8, incidence.data(),
      incidence.size(), nodes.data(), nodes.size(), &summary).status,
      c::SelfContactForceStatus::DuplicateEvent);
  EXPECT_EQ(summary.incidences, 71u);
  EXPECT_EQ(summary.touched_nodes, 73u);

  auto event = Event(11, 3, {0, 1, 2}, {3, 4, 5});
  EXPECT_EQ(c::BuildSelfContactForceIncidence(
      &event, 1, 6, incidence.data(), 5, nodes.data(), nodes.size(),
      &summary).status, c::SelfContactForceStatus::ResourceLimit);
  EXPECT_EQ(c::BuildSelfContactForceIncidence(
      &event, 1, 6, incidence.data(), incidence.size(), nodes.data(), 5,
      &summary).status, c::SelfContactForceStatus::ResourceLimit);
  auto* alias = reinterpret_cast<c::SelfContactForceIncidenceSummary*>(&event);
  EXPECT_EQ(c::BuildSelfContactForceIncidence(
      &event, 1, 6, incidence.data(), incidence.size(), nodes.data(),
      nodes.size(), alias).status, c::SelfContactForceStatus::InvalidInput);
}

}  // namespace self_contact_force_test
