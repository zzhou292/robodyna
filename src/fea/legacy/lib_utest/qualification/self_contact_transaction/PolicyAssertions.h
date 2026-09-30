// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included inside self_contact_transaction_cuda_test. The caller copies the
// authenticated accepted certificates before candidate sealing changes phase.
inline void CheckPolicyOwners(
    c::SelfContactCandidatePolicyView outcomes,
    const c::SelfContactCandidatePolicySummary& summary,
    const std::vector<sct::AcceptedEventCertificate>& owners) {
  ASSERT_TRUE(outcomes.complete);
  ASSERT_TRUE(summary.complete);
  ASSERT_EQ(outcomes.count, summary.outcomes);
  std::size_t vf = 0, ee = 0;
  for (std::size_t i = 0; i < outcomes.count; ++i) {
    const auto& row = outcomes.data[i];
    const bool is_vf = row.disposition ==
        c::SelfContactCandidateDisposition::RepresentedByAcceptedVertexFace;
    const bool is_ee = row.disposition ==
        c::SelfContactCandidateDisposition::RepresentedByAcceptedEdgeEdge;
    if (!is_vf && !is_ee) continue;
    SCOPED_TRACE(i);
    ASSERT_LT(row.accepted_event, owners.size());
    const auto& owner = owners[row.accepted_event];
    EXPECT_EQ(row.source_order, owner.event.source_order);
    EXPECT_EQ(owner.kind, is_vf ?
        sct::AcceptedEventCertificateKind::VertexFace :
        sct::AcceptedEventCertificateKind::EdgeEdge);
    EXPECT_EQ(owner.event.classification.kind, is_vf ?
        c::SelfContactPairKind::VertexFace : c::SelfContactPairKind::EdgeEdge);
    EXPECT_EQ(owner.event.classification.status, is_vf ?
        c::SelfContactPairStatus::AdmittedVertexFace :
        c::SelfContactPairStatus::AdmittedEdgeEdge);
    EXPECT_TRUE(owner.event.classification.active[0]);
    EXPECT_TRUE(owner.event.classification.active[1]);
    EXPECT_FALSE(owner.event.classification.excluded);
    is_vf ? ++vf : ++ee;
  }
  EXPECT_EQ(vf, summary.represented_by_accepted_vf);
  EXPECT_EQ(ee, summary.represented_by_accepted_ee);
}

// Invoked only by a failed expectation. Limit text rather than changing the
// simulation roster, and retain source identities for both published policy
// rows and the accepted force owners they could represent.
inline std::string DescribePolicyOwnerFailure(
    c::SelfContactCandidatePolicyView outcomes,
    const c::SelfContactCandidatePolicySummary& summary,
    const std::vector<sct::AcceptedEventCertificate>& owners,
    std::size_t active, std::size_t removing, std::size_t skipped) {
  std::ostringstream text;
  text.precision(17);
  text << "POLICY_OWNER_DIAGNOSTIC outcomes=" << summary.outcomes
       << " separated=" << summary.certified_separated
       << " same_rigid=" << summary.excluded_same_rigid_group
       << " local=" << summary.excluded_local_intersection
       << " vf=" << summary.represented_by_accepted_vf
       << " ee=" << summary.represented_by_accepted_ee
       << " exact_pairs=" << summary.exact_crossing_pairs
       << " linear_policy_pairs=" << summary.linear_policy_coverage_pairs
       << " linear_policy_separated=" << summary.linear_policy_certified_separated
       << " linear_policy_accepted=" << summary.linear_policy_accepted_coverage
       << " linear_policy_excluded=" << summary.linear_policy_exact_exclusion
       << " nonlinear_pairs=" << summary.nonlinear_subdivision_pairs
       << " active=" << active << " removing=" << removing
       << " skipped=" << skipped << " accepted_owners=" << owners.size();
  constexpr std::size_t TextRows = 64;
  for (std::size_t i = 0; outcomes.data && i < std::min(outcomes.count, TextRows); ++i) {
    const auto& row = outcomes.data[i];
    text << "\npolicy[" << i << "] disposition=" << static_cast<unsigned>(row.disposition)
         << " pair=" << row.pair.paths[0].parent_eid << ':' << row.pair.paths[0].local_facet
         << '/' << row.pair.paths[1].parent_eid << ':' << row.pair.paths[1].local_facet
         << " level=" << static_cast<unsigned>(row.pair.paths[0].level)
         << '/' << static_cast<unsigned>(row.pair.paths[1].level)
         << " event=" << row.accepted_event << " source_order=" << row.source_order;
  }
  for (std::size_t i = 0; i < std::min(owners.size(), TextRows); ++i) {
    const auto& owner = owners[i];
    const auto& feature = owner.discovery;
    text << "\nowner[" << i << "] kind=" << static_cast<unsigned>(owner.kind)
         << " source_order=" << owner.event.source_order
         << " pair=" << feature.triangles[0].parent_eid << ':' << feature.triangles[0].local_facet
         << '/' << feature.triangles[1].parent_eid << ':' << feature.triangles[1].local_facet
         << " distance=" << feature.distance_m
         << " half_thickness=" << owner.event.classification.reference_half_thickness_m[0]
         << '/' << owner.event.classification.reference_half_thickness_m[1]
         << " parent=" << owner.event.classification.parent[0]
         << '/' << owner.event.classification.parent[1];
  }
  if (outcomes.count > TextRows || owners.size() > TextRows)
    text << "\ntext rows limited to " << TextRows << " per roster";
  return text.str();
}

// Finite shell thickness can produce accepted penalty forces while the
// represented midsurfaces remain separated. Check the complete native facet
// product and every force owner's source pair, rather than requiring a
// particular VF/EE representative from a proof that needs no such owner.
inline void CheckSeparatedFacetProduct(
    c::SelfContactCandidatePolicyView outcomes,
    const c::SelfContactCandidatePolicySummary& summary,
    const std::vector<sct::AcceptedEventCertificate>& owners,
    const c::FixedContactFacetBinding& binding,
    std::size_t first_parent, std::size_t second_parent) {
  ASSERT_TRUE(outcomes.complete);
  ASSERT_TRUE(summary.complete);
  ASSERT_NE(outcomes.data, nullptr);
  const auto first_count = binding.facet_count(first_parent);
  const auto second_count = binding.facet_count(second_parent);
  ASSERT_GT(first_count, 0u);
  ASSERT_GT(second_count, 0u);
  ASSERT_LE(first_count, SIZE_MAX / second_count);
  ASSERT_EQ(outcomes.count, first_count * second_count);
  ASSERT_EQ(summary.outcomes, outcomes.count);
  ASSERT_EQ(summary.certified_separated, outcomes.count);
  EXPECT_EQ(summary.excluded_same_rigid_group, 0u);
  EXPECT_EQ(summary.excluded_local_intersection, 0u);
  EXPECT_EQ(summary.represented_by_accepted_vf, 0u);
  EXPECT_EQ(summary.represented_by_accepted_ee, 0u);
  for (std::size_t i = 0; i < outcomes.count; ++i) {
    SCOPED_TRACE(i);
    if (i) ASSERT_LT(sct::Compare(outcomes.data[i - 1].pair, outcomes.data[i].pair), 0);
    EXPECT_EQ(outcomes.data[i].disposition,
              c::SelfContactCandidateDisposition::CertifiedSeparated);
    EXPECT_EQ(outcomes.data[i].accepted_event, SIZE_MAX);
    EXPECT_EQ(outcomes.data[i].source_order, UINT64_MAX);
  }
  const auto pair_key = [](const c::FixedTriangleKey& a, const c::FixedTriangleKey& b) {
    c::RepresentedIntervalPairKey key{{
        {a.source_instance_id, a.parent_eid, a.level, a.local_facet},
        {b.source_instance_id, b.parent_eid, b.level, b.local_facet}}};
    if (sct::Compare(key.paths[0], key.paths[1]) > 0)
      std::swap(key.paths[0], key.paths[1]);
    return key;
  };
  const auto contains = [&](const c::RepresentedIntervalPairKey& key) {
    const auto* end = outcomes.data + outcomes.count;
    const auto* found = std::lower_bound(outcomes.data, end, key,
        [](const auto& row, const auto& value) { return sct::Compare(row.pair, value) < 0; });
    return found != end && sct::Compare(found->pair, key) == 0;
  };
  for (std::size_t first = 0; first < first_count; ++first)
    for (std::size_t second = 0; second < second_count; ++second) {
      c::FixedContactFacet a, b;
      ASSERT_EQ(binding.Describe(first_parent, first, &a).status, c::FixedContactFacetStatus::Ok);
      ASSERT_EQ(binding.Describe(second_parent, second, &b).status, c::FixedContactFacetStatus::Ok);
      EXPECT_TRUE(contains(pair_key(
          {a.source_instance_id, a.source.source_parent_id, a.level, a.local_facet},
          {b.source_instance_id, b.source.source_parent_id, b.level, b.local_facet})))
          << first << ',' << second;
    }
  ASSERT_FALSE(owners.empty());
  for (std::size_t owner = 0; owner < owners.size(); ++owner) {
    SCOPED_TRACE(owner);
    const auto& feature = owners[owner].discovery;
    EXPECT_TRUE(contains(pair_key(feature.triangles[0], feature.triangles[1])));
  }
}
