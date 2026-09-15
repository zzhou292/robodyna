// SPDX-License-Identifier: MIT
#include "../self_contact_current_regularity/Fixture.h"
#include "lib_src/collision/self_contact_transaction/Storage.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <numeric>
#include <vector>

namespace {
namespace c = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;

c::FixedTriangleKey Key(const c::FixedContactFacet& facet) {
  return {facet.source_instance_id, facet.source.source_parent_id,
          facet.level, facet.local_facet};
}

struct AcceptedFixture {
  current_regularity_test::Fixture fixture{0, true};
  c::SelfContactCurrentRegularityReceipt regularity_receipt;
  std::vector<c::FixedContactFacet> descriptors;
  std::vector<std::uint32_t> order;
  c::FixedTriangleFeatureCandidate admitted;

  AcceptedFixture()
      : descriptors(fixture.uses.facet_uses().size()),
        order(descriptors.size()) {
    fixture.InitializeRegularity();
    EXPECT_EQ(fixture.regularity.Certify(
        fixture.Positions(), fixture.Activity(),
        &regularity_receipt).status,
        c::SelfContactCurrentRegularityStatus::Ok);
    for (std::size_t facet = 0; facet < descriptors.size(); ++facet) {
      const auto& use = fixture.uses.facet_uses()[facet];
      const auto& parent = fixture.uses.parents()[use.parent];
      EXPECT_EQ(fixture.source.facets.Describe(
          parent.surface_parent, use.local_facet,
          descriptors.data()+facet).status,
          c::FixedContactFacetStatus::Ok);
    }
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
        [&](std::uint32_t first, std::uint32_t second) {
          return c::fixed_triangle_features::Compare(
              Key(descriptors[first]), Key(descriptors[second])) < 0;
        });

    bool found = false;
    constexpr std::array<double, 3> face_weights{{.25, .25, .5}};
    for (std::size_t vertex_facet = 0;
         vertex_facet < descriptors.size() && !found; ++vertex_facet) {
      const auto& vertex_facet_use =
          fixture.uses.facet_uses()[vertex_facet];
      for (unsigned local_vertex = 0;
           local_vertex < 3 && !found; ++local_vertex) {
        const auto vertex_use =
            vertex_facet_use.vertex_uses[local_vertex];
        for (std::size_t target = 0;
             target < descriptors.size() && !found; ++target) {
          if (fixture.uses.facet_uses()[target].parent ==
              vertex_facet_use.parent)
            continue;
          const auto face_point =
              fixture.source.FacePoint(
                  target, fixture.uses, face_weights);
          c::SelfContactPairClassification classification;
          const auto classified =
              fixture.uses.ClassifyVertexFace(
                  vertex_use, target, face_point,
                  fixture.Activity(), &classification);
          if (classified.status !=
                  c::SelfContactActiveUseStatus::Ok ||
              classification.status !=
                  c::SelfContactPairStatus::AdmittedVertexFace)
            continue;
          admitted.key.vertex_face.vertex =
              descriptors[vertex_facet].
                  vertex_keys[local_vertex];
          admitted.key.vertex_face.target.SetFace(
              Key(descriptors[target]));
          admitted.triangles[0] =
              Key(descriptors[vertex_facet]);
          admitted.triangles[1] = Key(descriptors[target]);
          admitted.local_features[0] = local_vertex;
          admitted.local_features[1] = 3;
          std::copy(face_weights.begin(), face_weights.end(),
                    admitted.face_weights);
          found = true;
        }
      }
    }
    EXPECT_TRUE(found);
  }

  c::SelfContactTransactionReport Build(
      const c::FixedTriangleFeatureCandidate* features,
      std::size_t feature_count,
      c::SelfContactForceEvent* events,
      sct::AcceptedEventCertificate* certificates,
      std::size_t capacity, std::size_t* count) {
    return sct::BuildAcceptedEvents(
        fixture.uses, fixture.regularity, regularity_receipt,
        {features, feature_count, true}, {nullptr, 0, true},
        descriptors.data(), order.data(), descriptors.size(),
        fixture.Activity(), events, certificates, capacity, count);
  }
};

TEST(SelfContactAcceptedEvents,
     OnePrivatePassRetainsLateFailurePriorityAndCountPublication) {
  AcceptedFixture fixture;
  const std::array<c::FixedTriangleFeatureCandidate, 2> late{
      fixture.admitted, fixture.admitted};
  auto invalid = late;
  invalid[1].local_features[1] = 2;

  std::array<c::SelfContactForceEvent, 2> events;
  std::array<sct::AcceptedEventCertificate, 2> certificates;
  std::memset(events.data(), 0xa5, sizeof(events));
  std::memset(certificates.data(), 0x5a, sizeof(certificates));
  const auto event_guard = events[1];
  const auto certificate_guard = certificates[1];
  std::size_t count = 777;
  const auto report = fixture.Build(
      invalid.data(), invalid.size(), events.data(),
      certificates.data(), 1, &count);
  EXPECT_EQ(report.status,
      c::SelfContactTransactionStatus::IdentityMismatch);
  EXPECT_EQ(report.candidate, 1u);
  EXPECT_EQ(count, 777u);
  EXPECT_EQ(std::memcmp(
      events.data()+1, &event_guard, sizeof(event_guard)), 0);
  EXPECT_EQ(std::memcmp(
      certificates.data()+1, &certificate_guard,
      sizeof(certificate_guard)), 0);
}

TEST(SelfContactAcceptedEvents,
     ShortPrivateCapacityCountsCompletelyWithoutOverwritingGuard) {
  AcceptedFixture fixture;
  const std::array<c::FixedTriangleFeatureCandidate, 2> features{
      fixture.admitted, fixture.admitted};

  std::array<c::SelfContactForceEvent, 2> events;
  std::array<sct::AcceptedEventCertificate, 2> certificates;
  std::memset(events.data(), 0xa5, sizeof(events));
  std::memset(certificates.data(), 0x5a, sizeof(certificates));
  const auto event_guard = events[1];
  const auto certificate_guard = certificates[1];
  std::size_t count = 888;
  const auto short_report = fixture.Build(
      features.data(), features.size(), events.data(),
      certificates.data(), 1, &count);
  EXPECT_EQ(short_report.status,
      c::SelfContactTransactionStatus::ResourceLimit);
  EXPECT_EQ(short_report.candidate, 2u);
  EXPECT_EQ(count, 888u);
  EXPECT_EQ(std::memcmp(
      events.data()+1, &event_guard, sizeof(event_guard)), 0);
  EXPECT_EQ(std::memcmp(
      certificates.data()+1, &certificate_guard,
      sizeof(certificate_guard)), 0);

  ASSERT_EQ(fixture.Build(
      features.data(), features.size(), events.data(),
      certificates.data(), 2, &count).status,
      c::SelfContactTransactionStatus::Ok);
  ASSERT_EQ(count, 2u);
  EXPECT_EQ(events[0].source_order, 0u);
  EXPECT_EQ(events[1].source_order, 1u);
}

}  // namespace
