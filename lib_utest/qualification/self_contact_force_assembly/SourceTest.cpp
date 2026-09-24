// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../self_contact_active_uses/Fixture.h"
#include "lib_src/collision/self_contact_force/Storage.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
namespace c = tlfea::contact;
namespace force = c::self_contact_force;
using S = c::SelfContactForceStatus;

void SameReport(const c::SelfContactForceReport& actual,
                const c::SelfContactForceReport& expected) {
  EXPECT_EQ(actual.status, expected.status);
  EXPECT_EQ(actual.event, expected.event);
  EXPECT_EQ(actual.source_order, expected.source_order);
  EXPECT_EQ(actual.node, expected.node);
  EXPECT_EQ(actual.pair_status, expected.pair_status);
  EXPECT_EQ(actual.owner_status, expected.owner_status);
  EXPECT_STREQ(actual.message, expected.message);
}

class SelfContactForceSource : public ::testing::Test {
 protected:
  active_use_test::Fixture fixture{2, false, true};
  c::SelfContactActiveUseBinding binding;
  std::vector<std::uint8_t> base, current;
  std::array<c::SelfContactForceEvent, 2> events;

  c::SelfContactActivityView Activity() const {
    return {base.data(), current.data(), base.size()};
  }

  void SetUp() override {
    ASSERT_EQ(binding.Initialize(fixture.facets).status,
              c::SelfContactActiveUseStatus::Ok);
    ASSERT_EQ(binding.parents().size(), 6u);
    base.assign(binding.parents().size(), 1);
    current = base;
    const auto vertex = fixture.VertexUse(100, binding, 10);
    ASSERT_NE(vertex, SIZE_MAX);
    const auto facet = fixture.RemoteFacet(
        101, binding.vertex_uses()[vertex].feature, binding);
    ASSERT_NE(facet, SIZE_MAX);
    auto& vf = events[0];
    vf.source_order = 701;
    vf.vertex_use = static_cast<std::uint32_t>(vertex);
    vf.facet_use = static_cast<std::uint32_t>(facet);
    vf.endpoints[0] = binding.vertex_uses()[vertex].point;
    vf.endpoints[1] = fixture.FacePoint(facet, binding);
    vf.feature.vertex_face.vertex = binding.vertex_uses()[vertex].key;
    const auto& facet_use = binding.facet_uses()[facet];
    vf.feature.vertex_face.target.SetFace({
        fixture.domain.source_instance_id(),
        binding.parents()[facet_use.parent].source.source_parent_id,
        fixture.facets.config().level, facet_use.local_facet});
    ASSERT_EQ(binding.ClassifyVertexFace(
        vertex, facet, vf.endpoints[1], Activity(), &vf.classification).status,
        c::SelfContactActiveUseStatus::Ok);
    ASSERT_EQ(vf.classification.status,
              c::SelfContactPairStatus::AdmittedVertexFace);

    auto& ee = events[1];
    ee.source_order = 702;
    ee.feature.SetEdgeEdge();
    for (unsigned side = 0; side < 2; ++side) {
      const auto parent = fixture.Parent(side ? 101 : 100, binding);
      const auto uses = binding.edge_uses();
      std::size_t edge = 0;
      while (edge < uses.size() && uses[edge].parent != parent) ++edge;
      ASSERT_LT(edge, uses.size());
      ee.edge_use[side] = static_cast<std::uint32_t>(edge);
      ee.feature.edge_edge.edges[side] = uses[edge].key;
      auto& point = ee.endpoints[side];
      point.count = uses[edge].endpoints[0].count;
      for (unsigned slot = 0; slot < point.count; ++slot) {
        point.nodes[slot] = uses[edge].endpoints[0].nodes[slot];
        point.weights[slot] = .5 * uses[edge].endpoints[0].weights[slot] +
                             .5 * uses[edge].endpoints[1].weights[slot];
      }
    }
    if (c::fixed_triangle_features::Compare(
            ee.feature.edge_edge.edges[0], ee.feature.edge_edge.edges[1]) > 0) {
      std::swap(ee.edge_use[0], ee.edge_use[1]);
      std::swap(ee.endpoints[0], ee.endpoints[1]);
      std::swap(ee.feature.edge_edge.edges[0], ee.feature.edge_edge.edges[1]);
    }
    ASSERT_EQ(binding.ClassifyEdgeEdge(
        ee.edge_use[0], ee.endpoints[0], ee.edge_use[1], ee.endpoints[1],
        c::SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum,
        Activity(), &ee.classification).status,
        c::SelfContactActiveUseStatus::Ok);
    ASSERT_EQ(ee.classification.status,
              c::SelfContactPairStatus::AdmittedEdgeEdge);
    for (const auto& event : events)
      for (unsigned side = 0; side < 2; ++side)
        ASSERT_NE(event.classification.parent[side], base.size() - 1);
  }

  c::SelfContactForceReport Legacy(
      c::SelfContactForceEventView batch) const {
    for (std::size_t event = 0; event < batch.count; ++event) {
      const auto report = force::ValidateEvent(
          binding, batch.data[event], Activity(), event);
      if (report.status != S::Ok) return report;
    }
    return {};
  }
};

TEST_F(SelfContactForceSource, CheckedBatchMatchesPublicVfEeQueriesAndOrder) {
  for (unsigned permutation = 0; permutation < 2; ++permutation) {
    const c::SelfContactForceEventView batch{events.data(), events.size()};
    const auto reference = Legacy(batch);
    ASSERT_EQ(reference.status, S::Ok) << reference.message;
    SameReport(force::ValidateEvents(binding, Activity(), batch), reference);
    std::reverse(events.begin(), events.end());
  }
  SameReport(force::ValidateEvents(binding, Activity(), {}), {});
}

TEST_F(SelfContactForceSource, ForgedVfEeEventsKeepExactCanonicalFirstFailure) {
  for (unsigned position = 0; position < events.size(); ++position) {
    for (unsigned mutation = 0; mutation < 26; ++mutation) {
      SCOPED_TRACE(::testing::Message() << "position=" << position
                                     << " mutation=" << mutation);
      auto forged = events;
      auto& event = forged[position];
      auto& pair = event.classification;
      const bool vf = event.feature.kind == c::FixedTriangleCandidateKind::VertexFace;
      switch (mutation) {
        case 0: pair.binding_identity = nullptr; break;
        case 1: pair.activity_base_identity = nullptr; break;
        case 2: pair.activity_current_identity = nullptr; break;
        case 3: ++pair.activity_parent_count; break;
        case 4: pair.kind = vf ? c::SelfContactPairKind::EdgeEdge
                              : c::SelfContactPairKind::VertexFace; break;
        case 5: pair.status = c::SelfContactPairStatus::ExcludedSameRigidGroup; break;
        case 6: pair.parent[0] = UINT32_MAX; break;
        case 7: pair.feature[1] = UINT32_MAX; break;
        case 8: pair.active[0] = false; break;
        case 9: pair.reference_half_thickness_m[1] = std::nextafter(
                    pair.reference_half_thickness_m[1], HUGE_VAL); break;
        case 10: pair.candidate_directed_area_m2.value = std::nextafter(
                     pair.candidate_directed_area_m2.value, HUGE_VAL); break;
        case 11: pair.admitted_force_area_m2.value = std::nextafter(
                     pair.admitted_force_area_m2.value, HUGE_VAL); break;
        case 12: ++pair.endpoint_support[1].nonzero_slots; break;
        case 13: pair.tied = c::SelfContactTiedStatus::
                     CompleteLocalSupportNeedsRuntimeActivity; break;
        case 14: pair.local_incidence = true; break;
        case 15: pair.excluded = true; break;
        case 16: event.endpoints[1].nodes[0] = UINT32_MAX; break;
        case 17: event.endpoints[0].weights[0] =
                     std::numeric_limits<double>::quiet_NaN(); break;
        case 18:
          if (vf) ++event.feature.vertex_face.vertex.source_instance_id;
          else ++event.feature.edge_edge.edges[0].endpoints[0].source_instance_id;
          break;
        case 19:
          if (vf) event.vertex_use = UINT32_MAX;
          else event.edge_use[0] = UINT32_MAX;
          break;
        case 20:
          if (vf) event.facet_use = UINT32_MAX;
          else event.edge_use[1] = UINT32_MAX;
          break;
        case 21:
          if (vf) event.edge_use[0] = 0;
          else event.vertex_use = 0;
          break;
        case 22: pair.edge_edge_case = vf
                     ? c::SelfContactEdgeEdgeCase::BoundaryVertexEdgeMinimum
                     : c::SelfContactEdgeEdgeCase::UnresolvedGeometricTie; break;
        case 23:
          if (vf) ++event.feature.vertex_face.target.face.parent_eid;
          else std::swap(event.feature.edge_edge.edges[0],
                         event.feature.edge_edge.edges[1]);
          break;
        case 24: pair.endpoint_support[0].status =
                     c::SelfContactSupportStatus::UnsupportedCinSecondary; break;
        case 25: pair.admitted_force_area_m2.error =
                     std::nextafter(pair.admitted_force_area_m2.error, HUGE_VAL); break;
      }
      const c::SelfContactForceEventView batch{forged.data(), forged.size()};
      const auto reference = Legacy(batch);
      ASSERT_NE(reference.status, S::Ok);
      EXPECT_EQ(reference.event, position);
      EXPECT_EQ(reference.source_order, event.source_order);
      SameReport(force::ValidateEvents(binding, Activity(), batch), reference);
    }
  }
  auto both = events;
  both[0].classification.binding_identity = nullptr;
  both[1].endpoints[1].nodes[0] = UINT32_MAX;
  const c::SelfContactForceEventView batch{both.data(), both.size()};
  const auto reference = Legacy(batch);
  ASSERT_EQ(reference.event, 0u);
  SameReport(force::ValidateEvents(binding, Activity(), batch), reference);
}

TEST_F(SelfContactForceSource, UnreferencedLastActivityRowRejectsEmptyAndNonemptyThenRetries) {
  const c::SelfContactForceReport expected{
      S::StaleAttempt, SIZE_MAX, UINT64_MAX, UINT32_MAX,
      c::SurfacePenaltyStatus::InvalidInput, tl::fea::NodalStatus::StaleTrial,
      "Supplied activity source is stale or invalid"};
  for (const auto count : {std::size_t{0}, events.size()}) {
    for (const auto values : {std::array<std::uint8_t, 2>{2, 1},
                              std::array<std::uint8_t, 2>{1, 2},
                              std::array<std::uint8_t, 2>{0, 1}}) {
      SCOPED_TRACE(::testing::Message() << "count=" << count
          << " base=" << unsigned(values[0]) << " current=" << unsigned(values[1]));
      base.back() = values[0];
      current.back() = values[1];
      const c::SelfContactForceEventView batch{count ? events.data() : nullptr, count};
      SameReport(force::ValidateEvents(binding, Activity(), batch), expected);
      EXPECT_EQ(base.back(), values[0]);
      EXPECT_EQ(current.back(), values[1]);
      base.back() = current.back() = 1;
      const auto retry = force::ValidateEvents(binding, Activity(), batch);
      ASSERT_EQ(retry.status, S::Ok) << retry.message;
    }
  }
}

TEST_F(SelfContactForceSource, InvalidBatchExtentsFailBeforeAnyEventRead) {
  EXPECT_EQ(force::ValidateEvents(binding, Activity(), {nullptr, 1}).status,
            S::InvalidInput);
  EXPECT_EQ(force::ValidateEvents(binding, Activity(),
                                 {events.data(), SIZE_MAX}).status,
            S::InvalidInput);
  c::SelfContactActiveUseBinding absent;
  EXPECT_EQ(force::ValidateEvents(absent, Activity(), {}).status,
            S::InvalidInput);
  const auto incomplete = c::SelfContactActivityView{
      base.data(), current.data(), base.size() - 1};
  EXPECT_EQ(force::ValidateEvents(binding, incomplete, {}).status,
            S::StaleAttempt);
  ASSERT_EQ(force::ValidateEvents(binding, Activity(),
                                 {events.data(), events.size()}).status, S::Ok);
}
}  // namespace
