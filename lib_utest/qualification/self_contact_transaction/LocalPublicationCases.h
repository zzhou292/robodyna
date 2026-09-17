// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Small coupon-only readback retained before SealCandidate's failure rollback.
// This does not change geometry, loads, ownership or the production report.
std::vector<c::CurrentFixedTriangle> CaptureLocalCouponFacets(
    Fixture& fixture, const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& prepared) {
  const auto nodes = fixture.rig.fixture.domain.node_count();
  EXPECT_LE(nodes, 256u);
  if (nodes > 256) return {};
  std::vector<double> fields(6 * nodes);
  fe::NodalPreparedView copied;
  if (!p::Good(fixture.rig.owner.CopyPrepared(
          token, {fields.data(), fields.data() + 3 * nodes, nodes}, &copied)))
    return {};
  EXPECT_TRUE(fe::trial_identity::SamePrepared(copied, prepared));
  std::vector<c::CurrentFixedTriangle> result;
  for (std::size_t parent = 0; parent < fixture.selection.size(); ++parent)
    for (unsigned local = 0; local < fixture.facets.facet_count(parent); ++local) {
      c::FixedContactFacet descriptor;
      const auto described = fixture.facets.Describe(parent, local, &descriptor);
      EXPECT_EQ(described.status, c::FixedContactFacetStatus::Ok);
      if (described.status != c::FixedContactFacetStatus::Ok) return {};
      c::CurrentFixedTriangle triangle;
      if (!Good(sct::EvaluateCompleteTriangles(
          &descriptor, 1, {fields.data(), static_cast<std::uint32_t>(nodes), 3, 1},
          &triangle))) return {};
      result.push_back(triangle);
    }
  return result;
}

std::string DescribeLocalCouponFailure(
    const std::vector<c::CurrentFixedTriangle>& triangles) {
  ::testing::Message text;
  for (const auto& triangle : triangles) {
    text << " facet=" << triangle.key.parent_eid << ':' << triangle.key.local_facet;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      const auto point = triangle.vertices[vertex];
      text << " vertex=" << triangle.vertex_keys[vertex].first
           << " xyz_bits=" << p::Bits(point.x) << ',' << p::Bits(point.y)
           << ',' << p::Bits(point.z);
    }
  }
  for (std::size_t first = 0; first < triangles.size(); ++first)
    for (std::size_t second = first + 1; second < triangles.size(); ++second) {
      if (triangles[first].key.parent_eid == triangles[second].key.parent_eid) continue;
      c::FixedTriangleFeatureTaskMask mask;
      if (c::BuildFixedTriangleFeatureTaskMask(triangles[first], triangles[second], &mask) !=
          c::FixedTriangleDiscoveryStatus::Ok) continue;
      c::FixedTriangleFeatureCandidate features[15];
      c::fixed_triangle_features::PairFeatureResult result;
      const auto status = c::fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
          triangles[first], triangles[second], mask, features, 15, &result);
      text << " native_pair=" << first << ',' << second << " status=" << unsigned(status)
           << " task=" << result.input_task << " reason=" << unsigned(result.arithmetic_reason);
    }
  return text.GetString();
}

void CheckSecondIntervalRepresentationCoordinates(
    const std::vector<c::CurrentFixedTriangle>& triangles) {
  // Exact prepared-coordinate words from tests-2's original three-attempt
  // sequence: discard epoch0, commit epoch0, then prepare epoch1. The old log
  // omitted epoch; it must not be mistaken for a first-interval failure.
  struct Row { std::uint64_t source, xyz[3]; };
  constexpr Row expected[]{
      {10,{4269727632817760733ull,4269950757533192212ull,4263611001487772243ull}},
      {11,{4585925428558828667ull,13453662799716478440ull,13487346986433769302ull}},
      {12,{4585925428558828667ull,4581421828931458171ull,0}},
      {13,{0,4581421828931458171ull,0}},
      {14,{4587366580439587226ull,4576918229304087675ull,0}}};
  for (const auto& triangle : triangles)
    for (unsigned vertex=0;vertex<3;++vertex) {
      const auto source=triangle.vertex_keys[vertex].first;
      const Row* row=nullptr;
      for(const auto& candidate:expected) if(candidate.source==source) row=&candidate;
      ASSERT_NE(row,nullptr);
      const auto point=triangle.vertices[vertex];
      EXPECT_EQ(p::Bits(point.x),row->xyz[0]) << "source=" << source;
      EXPECT_EQ(p::Bits(point.y),row->xyz[1]) << "source=" << source;
      EXPECT_EQ(p::Bits(point.z),row->xyz[2]) << "source=" << source;
    }
}

// Original unconstrained T3(102)/Q4(103) share source nodes 11 and 12.
// The first physical interval commits a continuously local policy. Native
// structural/CIN response in the second interval reaches a real interior
// edge parameter whose complementary weight rounds to one. Preserve that
// representation limit and automatic rollback/retry without changing loads,
// boundary conditions, initial geometry, materials or the physical step.
TEST(SelfContactTransactionCuda,
     AdjacentLocalFirstIntervalCommitsThenSecondIntervalRejectsRetryAtomically) {
  Fixture fixture(true);
  // Constructor true keeps original adjacent geometry. Selecting both
  // existing parents adds no synthetic topology or new structural owner.
  fixture.single_parent = false;
  for (const std::uint64_t source_node : {10u, 11u, 12u, 13u, 14u}) {
    const auto node = fixture.rig.fixture.domain.Find(source_node);
    ASSERT_LT(node, fixture.rig.fixture.domain.node_count());
    ASSERT_EQ(fixture.rig.fixture.fixed[node], 0u);
    for (unsigned component = 0; component < 3; ++component)
      ASSERT_EQ(fixture.rig.fixture.v[3 * node + component], 0);
  }
  ASSERT_TRUE(fixture.Initialize());
  ASSERT_EQ(fixture.uses.parents().size(), 2u);
  {
    SCOPED_TRACE("accepted_epoch=0 first physical interval");
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token,assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner,token,assembly,&accepted)));
    EXPECT_GT(accepted.local_masked_tasks(),0u);
    EXPECT_EQ(accepted.diagnostics().event_count,0u);
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token,assembly,prepared,common));
    c::SelfContactTransactionReceipt receipt;
    ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
        fixture.rig.owner,token,common,prepared,accepted,&receipt)));
    ASSERT_TRUE(receipt.valid());
    const auto& policy=receipt.policy_summary();
    EXPECT_TRUE(policy.complete);
    EXPECT_GT(policy.excluded_local_intersection,0u);
    EXPECT_GT(policy.linear_policy_exact_exclusion,0u);
    EXPECT_EQ(policy.excluded_same_rigid_group,0u);
    EXPECT_EQ(policy.linear_policy_unresolved,0u);
    EXPECT_EQ(policy.nonlinear_subdivision_unresolved,0u);
    ASSERT_TRUE(fixture.Commit(token,prepared,common,receipt));
    ASSERT_EQ(fixture.rig.owner.accepted().epoch,1u);
  }
  p::Snapshot before, after;
  ASSERT_TRUE(fixture.rig.Read(before));
  std::string first_geometry;
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    SCOPED_TRACE(::testing::Message() << "accepted_epoch=1 retry=" << attempt);
    ASSERT_EQ(fixture.rig.owner.accepted().epoch, 1u);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    EXPECT_GT(accepted.local_masked_tasks(), 0u);
    EXPECT_EQ(accepted.diagnostics().event_count, 0u);
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    const auto prepared_facets = CaptureLocalCouponFacets(fixture, token, prepared);
    ASSERT_EQ(prepared_facets.size(), 3u);
    ASSERT_NO_FATAL_FAILURE(CheckSecondIntervalRepresentationCoordinates(prepared_facets));
    const auto geometry = DescribeLocalCouponFailure(prepared_facets);
    if (attempt == 0) first_geometry = geometry;
    else EXPECT_EQ(geometry, first_geometry);
    c::SelfContactTransactionReceipt receipt;
    const auto sealed = fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &receipt);
    EXPECT_EQ(sealed.status, c::SelfContactTransactionStatus::DiscoveryFailure)
        << geometry;
    EXPECT_EQ(sealed.discovery_status, c::FixedTriangleDiscoveryStatus::NonFiniteResult);
    EXPECT_EQ(sealed.discovery_task, 1u);
    EXPECT_EQ(sealed.discovery_reason,
              c::FixedTriangleArithmeticReason::EdgeInteriorRepresentation);
    EXPECT_EQ(sealed.candidate, SIZE_MAX);
    EXPECT_EQ(sealed.pair, 0u);
    EXPECT_FALSE(receipt.valid());
    EXPECT_FALSE(accepted.valid());
    EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);
    const auto outcomes = fixture.transaction.policy_outcomes();
    EXPECT_FALSE(outcomes.complete);
    // SealCandidate itself must roll back every participant. No caller-side
    // Discard is allowed to repair a failed transaction before this snapshot.
    ASSERT_TRUE(fixture.rig.Read(after));
    p::Exact(before, after);
  }
}
