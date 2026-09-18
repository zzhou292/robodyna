// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

// Included after LocalPublicationCases.h; reuse its native prepared-facet
// readback and the owning transaction/publication fixture, without changing it.
void CheckTranslatedLocalPolicy(const c::SelfContactTransactionReceipt& receipt) {
  ASSERT_TRUE(receipt.valid());
  const auto& policy = receipt.policy_summary();
  ASSERT_TRUE(policy.complete);
  EXPECT_GT(policy.excluded_local_intersection, 0u);
  EXPECT_GT(policy.exact_crossing_pairs, 0u);
  EXPECT_EQ(policy.exact_crossing_work, policy.exact_crossing_pairs);
  EXPECT_EQ(policy.linear_policy_coverage_pairs, 0u);
  EXPECT_EQ(policy.linear_policy_coverage_work, 0u);
  EXPECT_EQ(policy.linear_policy_exact_exclusion, 0u);
  EXPECT_EQ(policy.linear_policy_unresolved, 0u);
  EXPECT_EQ(policy.nonlinear_subdivision_unresolved, 0u);
}

void CheckDeclaredLocalMotion(Fixture& fixture, const fe::NodalTrialToken& token,
                              const fe::NodalPreparedView& prepared,
                              double translation_z) {
  const auto triangles = CaptureLocalCouponFacets(fixture, token, prepared);
  ASSERT_EQ(triangles.size(), 3u);
  for (const auto& triangle : triangles) {
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
      const auto source = triangle.vertex_keys[vertex].first;
      const auto node = fixture.rig.fixture.domain.Find(source);
      ASSERT_LT(node, fixture.rig.fixture.domain.node_count());
      EXPECT_EQ(p::Bits(triangle.vertices[vertex].x),
                p::Bits(fixture.rig.fixture.x[3 * node]));
      EXPECT_EQ(p::Bits(triangle.vertices[vertex].y),
                p::Bits(fixture.rig.fixture.x[3 * node + 1]));
      EXPECT_EQ(p::Bits(triangle.vertices[vertex].z),
                p::Bits(fixture.rig.fixture.x[3 * node + 2] + translation_z));
    }
  }
}

TEST(SelfContactTransactionCuda, OriginalAdjacentStaticAndUniformStartupUseTheirActualLocalProofs) {
  for (const double speed_z : {0.0, 1.0}) {
    SCOPED_TRACE(::testing::Message() << "declared initial contact velocity z=" << speed_z);
    fe::ShellBatchStartup startup;
    if (speed_z != 0)
      startup = {fe::ShellBatchStartupKind::ReferenceUniformTranslation, {0, 0, speed_z}};
    Fixture fixture(true, false, 2.5, p::ContactConstraintLayout::Legacy,
                    0, 0, .05, startup);
    fixture.single_parent = false;
    // Declare one complete reference uniform translation to the owner and
    // every participant/publication. Contact z starts exactly at zero, so
    // speed*H is an exact dyadic displacement; source geometry is unchanged.
    for (const std::uint64_t source : {10u, 11u, 12u, 13u, 14u}) {
      const auto node = fixture.rig.fixture.domain.Find(source);
      ASSERT_LT(node, fixture.rig.fixture.domain.node_count());
      ASSERT_EQ(fixture.rig.fixture.x[3 * node + 2], 0);
      ASSERT_EQ(fixture.rig.fixture.fixed[node], 0u);
      ASSERT_EQ(fixture.rig.fixture.v[3 * node + 2], speed_z);
    }
    ASSERT_TRUE(fixture.Initialize());
    p::Snapshot initial, discarded;
    ASSERT_TRUE(fixture.rig.Read(initial));
    std::vector<std::uint64_t> first_policy;
    std::vector<std::uint64_t> first_geometry;
    for (unsigned retry = 0; retry < 2; ++retry) {
      SCOPED_TRACE(::testing::Message() << "accepted_epoch=0 retry=" << retry);
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
      std::vector<std::uint64_t> geometry_bits;
      if (speed_z == 0) {
        ASSERT_NO_FATAL_FAILURE(CheckDeclaredLocalMotion(fixture, token, prepared, 0));
      } else {
        const auto triangles = CaptureLocalCouponFacets(fixture, token, prepared);
        ASSERT_EQ(triangles.size(), 3u);
        std::size_t changed_in_plane = 0;
        for (const auto& triangle : triangles)
          for (unsigned vertex = 0; vertex < 3; ++vertex) {
            const auto node = fixture.rig.fixture.domain.Find(triangle.vertex_keys[vertex].first);
            ASSERT_LT(node, fixture.rig.fixture.domain.node_count());
            const auto point = triangle.vertices[vertex];
            EXPECT_EQ(p::Bits(point.z), p::Bits(p::H));
            changed_in_plane += p::Bits(point.x) != p::Bits(fixture.rig.fixture.x[3 * node]);
            changed_in_plane += p::Bits(point.y) != p::Bits(fixture.rig.fixture.x[3 * node + 1]);
            Append(point, &geometry_bits);
          }
        // Real CIN arithmetic produces tiny in-plane changes. Preserve its
        // exact repeated output and require the general continuous coverage
        // path, rather than calling this an exact whole-body translation.
        EXPECT_GT(changed_in_plane, 0u);
      }
      c::SelfContactTransactionReceipt receipt;
      ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
          fixture.rig.owner, token, common, prepared, accepted, &receipt)));
      if (speed_z == 0) {
        ASSERT_NO_FATAL_FAILURE(CheckTranslatedLocalPolicy(receipt));
      } else {
        const auto& policy = receipt.policy_summary();
        ASSERT_TRUE(policy.complete);
        EXPECT_GT(policy.excluded_local_intersection, 0u);
        EXPECT_GT(policy.exact_crossing_work, policy.exact_crossing_pairs);
        EXPECT_GT(policy.linear_policy_coverage_pairs, 0u);
        EXPECT_GT(policy.linear_policy_coverage_work, 0u);
        EXPECT_GT(policy.linear_policy_exact_exclusion, 0u);
        EXPECT_EQ(policy.linear_policy_unresolved, 0u);
        EXPECT_EQ(policy.nonlinear_subdivision_unresolved, 0u);
      }
      const auto policy = PolicySummaryBits(receipt.policy_summary());
      if (!retry) {
        first_policy = policy;
        first_geometry = geometry_bits;
        fixture.Discard();
        EXPECT_FALSE(receipt.valid());
        ASSERT_TRUE(fixture.rig.Read(discarded));
        p::Exact(initial, discarded);
      } else {
        EXPECT_EQ(policy, first_policy);
        EXPECT_EQ(geometry_bits, first_geometry);
        ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
        EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);
        EXPECT_EQ(p::Bits(fixture.rig.owner.accepted().time), p::Bits(p::H));
      }
    }
  }
}

TEST(SelfContactTransactionCuda, ReferenceOnsetLocalContactRetainsPositiveForceAndFreeSti) {
  const double apex_x = std::nextafter(.04 + .0005, .04);
  Fixture fixture(true, false, 2.5, p::ContactConstraintLayout::Legacy,
                  0, 0, apex_x, {}, true);
  fixture.single_parent = false;
  const auto apex = fixture.rig.fixture.domain.Find(14);
  ASSERT_LT(apex, fixture.rig.fixture.domain.node_count());
  ASSERT_EQ(p::Bits(fixture.rig.fixture.x[3 * apex]), p::Bits(apex_x));
  // Explicit contact-onset reference geometry: exact penetration 15/2^61 m.
  // Native positive force is small enough that unchanged free dynamics may
  // round the first represented positions back to their accepted values.
  // The exact stationary-geometry assertion below must prove this; no force,
  // material, mass, step, velocity or position correction is applied.
  // Contact Q4 nodes 20--23 are disjoint from actual QEPH/CIN masters 10--13.
  ASSERT_EQ(fixture.rig.fixture.WitnessCount(), 2u);
  for (const std::uint64_t source : {10u, 11u, 12u, 13u, 14u, 20u, 21u, 22u, 23u}) {
    const auto node = fixture.rig.fixture.domain.Find(source);
    ASSERT_LT(node, fixture.rig.fixture.domain.node_count());
    ASSERT_EQ(fixture.rig.fixture.fixed[node], 0u);
    ASSERT_GT(fixture.rig.fixture.m[node], 0);
    ASSERT_GT(fixture.rig.fixture.im[node], 0);
  }
  ASSERT_TRUE(fixture.Initialize());
  p::Snapshot initial, discarded;
  ASSERT_TRUE(fixture.rig.Read(initial));
  std::vector<std::uint64_t> first_fields, first_policy;
  for (unsigned retry = 0; retry < 2; ++retry) {
    SCOPED_TRACE(::testing::Message() << "free onset local accepted_epoch=0 retry=" << retry);
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(fixture.rig.Begin(token, assembly));
    fe::NodalCinAssemblyView cin;
    ASSERT_TRUE(p::Good(fixture.rig.owner.BorrowCinAssembly(token, &cin)));
    AssemblyFields before(fixture.rig.fixture.domain.node_count()), after(before.nodes);
    ASSERT_TRUE(before.Read(assembly, cin));
    c::SelfContactAcceptedAssemblyReceipt accepted;
    ASSERT_TRUE(Good(fixture.transaction.AssembleAccepted(
        fixture.rig.owner, token, assembly, &accepted)));
    const auto& diagnostics = accepted.diagnostics();
    ASSERT_TRUE(diagnostics.valid);
    EXPECT_GT(accepted.local_masked_tasks(), 0u);
    ASSERT_GT(diagnostics.event_count, 0u);
    EXPECT_GT(diagnostics.active_count, 0u);
    EXPECT_GT(diagnostics.maximum_force_norm_n, 0);
    EXPECT_GT(diagnostics.potential_j, 0);
    EXPECT_GT(diagnostics.maximum_represented_stiffness_n_m, 0);
    EXPECT_GT(diagnostics.maximum_sti_diagonal_n_m, 0);
    ASSERT_TRUE(after.Read(assembly, cin));
    double force_change = 0, sti_change = 0;
    for (std::size_t node = 0; node < before.nodes; ++node) {
      for (unsigned component = 0; component < 3; ++component)
        force_change += std::abs(after.values[component * before.nodes + node] -
                                 before.values[component * before.nodes + node]);
      EXPECT_GE(after.values[6 * before.nodes + node], before.values[6 * before.nodes + node]);
      sti_change += after.values[6 * before.nodes + node] - before.values[6 * before.nodes + node];
    }
    EXPECT_GT(force_change, 0);
    EXPECT_GT(sti_change, 0);
    const auto owners = sct::QualificationAccess::AcceptedCertificates(fixture.transaction);
    ASSERT_TRUE(owners.complete);
    EXPECT_EQ(owners.count, diagnostics.event_count);
    std::vector<c::RepresentedIntervalPairKey> positive_owner_pairs;
    for (std::size_t index = 0; index < owners.count; ++index) {
      const auto& owner = owners.data[index];
      const auto& feature = owner.discovery;
      const auto& classification = owner.event.classification;
      const auto& first = feature.triangles[0];
      const auto& second = feature.triangles[1];
      const bool original_pair =
          (first.parent_eid == 102 && second.parent_eid == 103) ||
          (first.parent_eid == 103 && second.parent_eid == 102);
      const bool admitted =
          classification.status == c::SelfContactPairStatus::AdmittedVertexFace ||
          classification.status == c::SelfContactPairStatus::AdmittedEdgeEdge;
      const double thickness = classification.reference_half_thickness_m[0] +
                               classification.reference_half_thickness_m[1];
      if (!original_pair || !admitted || !(feature.distance_m > 0) ||
          !(feature.distance_m < thickness))
        continue;
      EXPECT_TRUE(classification.active[0]);
      EXPECT_TRUE(classification.active[1]);
      EXPECT_FALSE(classification.excluded);
      EXPECT_FALSE(classification.local_incidence);
      c::RepresentedIntervalPairKey key{{
          {first.source_instance_id, first.parent_eid, first.level, first.local_facet},
          {second.source_instance_id, second.parent_eid, second.level, second.local_facet}}};
      if (sct::Compare(key.paths[1], key.paths[0]) < 0)
        std::swap(key.paths[0], key.paths[1]);
      positive_owner_pairs.push_back(key);
    }
    ASSERT_FALSE(positive_owner_pairs.empty());
    fe::NodalPreparedView prepared;
    fe::ShellPhysicalDiagnostics common;
    ASSERT_TRUE(fixture.Prepare(token, assembly, prepared, common));
    ASSERT_NO_FATAL_FAILURE(CheckDeclaredLocalMotion(fixture, token, prepared, 0));
    // The physical kick must reach the free owner even when the position sum
    // rounds back to the same represented coordinate at this onset scale.
    const auto nodes = fixture.rig.fixture.domain.node_count();
    ASSERT_LE(nodes, 256u);
    std::vector<double> kinematics(6 * nodes);
    fe::NodalPreparedView copied;
    ASSERT_TRUE(p::Good(fixture.rig.owner.CopyPrepared(
        token, {kinematics.data(), kinematics.data() + 3 * nodes, nodes}, &copied)));
    EXPECT_TRUE(fe::trial_identity::SamePrepared(copied, prepared));
    EXPECT_GT(kinematics[3 * nodes + 3 * apex], 0);
    c::SelfContactTransactionReceipt receipt;
    ASSERT_TRUE(Good(fixture.transaction.SealCandidate(
        fixture.rig.owner, token, common, prepared, accepted, &receipt)));
    ASSERT_NO_FATAL_FAILURE(CheckTranslatedLocalPolicy(receipt));
    const auto outcomes = fixture.transaction.policy_outcomes();
    ASSERT_TRUE(outcomes.complete);
    std::size_t force_pairs_on_local_path = 0;
    for (const auto& key : positive_owner_pairs)
      for (std::size_t index = 0; index < outcomes.count; ++index)
        if (sct::Compare(key, outcomes.data[index].pair) == 0 &&
            outcomes.data[index].disposition ==
                c::SelfContactCandidateDisposition::ExcludedLocalIntersection)
          ++force_pairs_on_local_path;
    // The positive unmasked accepted owner and the continuous local outcome
    // must be the SAME canonical pair, not merely coexist elsewhere in a run.
    EXPECT_GT(force_pairs_on_local_path, 0u);
    const auto fields = FieldBits(after);
    const auto policy = PolicySummaryBits(receipt.policy_summary());
    if (!retry) {
      first_fields = fields;
      first_policy = policy;
      fixture.Discard();
      EXPECT_FALSE(receipt.valid());
      ASSERT_TRUE(fixture.rig.Read(discarded));
      p::Exact(initial, discarded);
    } else {
      EXPECT_EQ(fields, first_fields);
      EXPECT_EQ(policy, first_policy);
      ASSERT_TRUE(fixture.Commit(token, prepared, common, receipt));
      EXPECT_EQ(fixture.rig.owner.accepted().epoch, 1u);
      EXPECT_EQ(p::Bits(fixture.rig.owner.accepted().time), p::Bits(p::H));
    }
  }
}
