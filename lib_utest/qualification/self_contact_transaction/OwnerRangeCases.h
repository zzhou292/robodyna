// SPDX-License-Identifier: MIT
#pragma once

// Included after ValueTest.cpp's independent native feature/owner fixtures.
// Expected coverage always uses the original raw-array full scan, never a
// second implementation of the prefix range search.

static_assert(!std::is_default_constructible_v<sct::FinalizedCoverageLedger>);
static_assert(!std::is_copy_constructible_v<sct::FinalizedCoverageLedger>);
static_assert(!std::is_move_constructible_v<sct::FinalizedCoverageLedger>);
static_assert(!std::is_constructible_v<sct::FinalizedCoverageLedger,
    const sct::AcceptedEventCertificate*, std::size_t>);

void SortOwnerLedger(std::vector<sct::AcceptedEventCertificate>* ledger) {
  std::stable_sort(ledger->begin(), ledger->end(), [](const auto& a, const auto& b) {
    return c::CompareSelfContactForceEventIdentity(a.event, b.event) < 0;
  });
}

struct OwnerRangeGeometry {
  c::CurrentFixedTriangle first = Triangle(
      10, {1, 2, 3}, {{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}}});
  c::CurrentFixedTriangle second = Triangle(
      20, {4, 5, 6}, {{{.5, .5, .03125}, {1.5, .5, .03125}, {.5, 1.5, .03125}}});
};

sct::NonlinearSeparationResult CompareOwnerCoverage(
    const c::CurrentFixedTriangle& a, const c::CurrentFixedTriangle& a_next,
    const c::CurrentFixedTriangle& b, const c::CurrentFixedTriangle& b_next,
    const sct::FacetQuadraticCoefficients& quadratic,
    const std::vector<sct::AcceptedEventCertificate>& certificates,
    std::size_t work = 255, unsigned depth = 8, double thickness = .1) {
  const auto& access = sct::FinalizedCoverageLedgerTestAccess::Checked(
      certificates.data(), certificates.size());
  const auto original = sct::CertifyQuadraticFacetCoverage(
      a, a_next, Quadratic(0), thickness, b, b_next, quadratic, thickness, 1,
      certificates.data(), certificates.size(), work, depth);
  const auto ranged = sct::CertifyQuadraticFacetCoverage(
      a, a_next, Quadratic(0), thickness, b, b_next, quadratic, thickness, 1,
      access, work, depth);
  policy_exclusion_test::ExpectResult(original, ranged);
  const auto original_policy = sct::CertifyQuadraticFacetPolicyCoverage(
      a, a_next, Quadratic(0), thickness, b, b_next, quadratic, thickness, 1,
      certificates.data(), certificates.size(), nullptr, 0, work, depth);
  const auto ranged_policy = sct::CertifyQuadraticFacetPolicyCoverage(
      a, a_next, Quadratic(0), thickness, b, b_next, quadratic, thickness, 1,
      access, nullptr, 0, work, depth);
  policy_exclusion_test::ExpectResult(original_policy, ranged_policy);
  return ranged;
}

TEST(SelfContactOwnerRanges, BothOrientationsAndEveryNativeFeatureUseFullScanOracle) {
  const OwnerRangeGeometry geometry;
  auto edge_target = geometry.second;
  edge_target.vertices[0] = {-1, .5, .03125};
  edge_target.vertices[1] = {1, .5, .03125};
  edge_target.vertices[2] = {0, 1.5, .03125};
  std::array<bool, 3> vf_strata{};
  bool saw_ee = false;
  std::size_t covered = 0;
  for (const auto& second : {geometry.second, edge_target}) {
    const auto features = DiscoverPreparedPair(geometry.first, second);
    for (std::size_t i = 0; i < features.count; ++i) {
      const auto& feature = features.values[i];
      if (feature.key.kind == c::FixedTriangleCandidateKind::VertexFace)
        vf_strata[static_cast<unsigned>(feature.key.vertex_face.target.kind)] = true;
      else saw_ee = true;
      std::vector<sct::AcceptedEventCertificate> ledger{AcceptedCertificate(feature)};
      ledger[0].event.source_order = 17;
      const auto result = CompareOwnerCoverage(
          geometry.first, geometry.first, second, second, Quadratic(0), ledger);
      covered += result.status == sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage;
      CompareOwnerCoverage(second, second, geometry.first, geometry.first,
                           Quadratic(0), ledger);
    }
  }
  EXPECT_TRUE(saw_ee);
  for (bool seen : vf_strata) EXPECT_TRUE(seen);
  EXPECT_GT(covered, 0u);
}

TEST(SelfContactOwnerRanges, SeamsDistinctOwnersAndOriginalCertificateOrdinalSurviveRanges) {
  const OwnerRangeGeometry geometry;
  const auto features = DiscoverPreparedPair(geometry.first, geometry.second);
  std::vector<sct::AcceptedEventCertificate> ledger;
  for (std::size_t i = 0; i < features.count; ++i) {
    auto owner = AcceptedCertificate(features.values[i]);
    owner.event.source_order = 100 + i;
    ledger.push_back(owner);
    auto seam = owner;
    seam.discovery.triangles[0].parent_eid = 9;
    seam.discovery.triangles[1].parent_eid = 19;
    seam.event.classification.parent[0] += 100;
    seam.event.source_order = i;
    ledger.push_back(seam);
  }
  // Unrelated owners can sort before valid owners. The output must preserve
  // absolute ledger ordinals, not remap a short copied owner list to zero.
  for (std::size_t i = 0; i < 512; ++i) {
    auto unrelated = AcceptedCertificate(features.values[i % features.count]);
    auto& key = unrelated.event.feature;
    if (key.kind == c::FixedTriangleCandidateKind::VertexFace)
      key.vertex_face.vertex.source_instance_id += 10 + i;
    else
      key.edge_edge.edges[0].endpoints[0].source_instance_id += 10 + i;
    unrelated.discovery.key = key;
    unrelated.event.source_order = 1000 + i;
    ledger.push_back(unrelated);
  }
  SortOwnerLedger(&ledger);
  CompareOwnerCoverage(geometry.first, geometry.first,
      geometry.second, geometry.second, Quadratic(0), ledger);
  const c::CurrentFixedTriangle pair[]{geometry.first, geometry.second};
  const auto access = sct::FinalizedCoverageLedgerTestAccess::Checked(ledger.data(), ledger.size());
  const auto ranges = access.ForPair(pair);
  std::size_t visits = 0, previous_end = 0;
  for (std::size_t i = 0; i < ranges.count; ++i) {
    EXPECT_LT(ranges.values[i].begin, ranges.values[i].end);
    EXPECT_GE(ranges.values[i].begin, previous_end);
    previous_end = ranges.values[i].end;
    visits += ranges.values[i].end - ranges.values[i].begin;
  }
  EXPECT_LT(visits, ledger.size());
  EXPECT_LT(visits + ranges.comparisons, ledger.size());
  EXPECT_LE(ranges.count, 12u);
  EXPECT_GT(ranges.comparisons, 0u);
}

TEST(SelfContactOwnerRanges, DuplicateSourceOrderAndSixtyFifthOwnerRetainAmbiguity) {
  const OwnerRangeGeometry geometry;
  const auto native = DiscoverPreparedPair(geometry.first, geometry.second);
  const auto base = AcceptedCertificate(FirstEdgeEdge(native));
  for (const std::size_t count : {0u, 1u, 64u, 65u}) {
    std::vector<sct::AcceptedEventCertificate> ledger(count, base);
    for (std::size_t i = 0; i < count; ++i) {
      ledger[i].event.classification.parent[0] = 100 + i;
      ledger[i].event.source_order = count - i;
    }
    SortOwnerLedger(&ledger);
    const auto result = CompareOwnerCoverage(geometry.first, geometry.first,
        geometry.second, geometry.second, Quadratic(0), ledger);
    if (count == 65) EXPECT_EQ(result.status, sct::NonlinearSeparationStatus::OwnerAmbiguity);
    if (count == 64) EXPECT_NE(result.status, sct::NonlinearSeparationStatus::OwnerAmbiguity);
  }
  std::vector<sct::AcceptedEventCertificate> duplicate{base, base};
  EXPECT_EQ(CompareOwnerCoverage(geometry.first, geometry.first,
      geometry.second, geometry.second, Quadratic(0), duplicate).status,
      sct::NonlinearSeparationStatus::OwnerAmbiguity);
}

TEST(SelfContactOwnerRanges, InvalidOwnersAndExactKeyFieldsCannotAcquireCoverage) {
  const OwnerRangeGeometry geometry;
  const auto native = DiscoverPreparedPair(geometry.first, geometry.second);
  const auto base = AcceptedCertificate(FirstEdgeEdge(native));
  for (unsigned mutation = 0; mutation < 9; ++mutation) {
    auto owner = base;
    if (mutation == 0) owner.event.classification.active[0] = false;
    if (mutation == 1) owner.event.classification.local_incidence = true;
    if (mutation == 2) owner.event.classification.excluded = true;
    if (mutation == 3) owner.event.source_order = UINT64_MAX;
    if (mutation == 4) owner.discovery.key.edge_edge.edges[0].parent_eid++;
    if (mutation == 5) owner.event.feature.edge_edge.edges[0].endpoints[0].source_instance_id++;
    if (mutation == 6) owner.event.feature.edge_edge.edges[0].parent_boundary =
        !owner.event.feature.edge_edge.edges[0].parent_boundary;
    if (mutation == 7) owner.event.feature.edge_edge.edges[0].parent_eid++;
    if (mutation == 8) owner.discovery.edge_parameters[0] = std::numeric_limits<double>::quiet_NaN();
    if (mutation >= 5 && mutation <= 7) owner.discovery.key = owner.event.feature;
    SCOPED_TRACE(mutation);
    CompareOwnerCoverage(geometry.first, geometry.first,
        geometry.second, geometry.second, Quadratic(0), {owner});
  }
}

TEST(SelfContactOwnerRanges, WorkDepthInvalidGeometryAndRetryHaveIdenticalReports) {
  const auto first = Triangle(10, {1, 2, 3}, {{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}});
  const auto base = Triangle(20, {4, 5, 6}, {{{-.075, 0, .05}, {1.925, 0, .05}, {-.075, 2, .05}}});
  auto next = base;
  for (auto& vertex : next.vertices) vertex.x += .15;
  auto curved = Quadratic(0);
  for (unsigned vertex = 0; vertex < 3; ++vertex) curved.q[vertex][1] = {-1.2, -1.2};
  const auto native = DiscoverPreparedPair(first, base);
  const std::vector<sct::AcceptedEventCertificate> ledger{AcceptedCertificate(FirstEdgeEdge(native))};
  for (std::size_t work : {0u, 1u, 255u})
    for (unsigned depth : {0u, 8u, 53u})
      CompareOwnerCoverage(first, first, base, next, curved, ledger, work, depth);
  auto invalid = next;
  invalid.key.parent_eid++;
  CompareOwnerCoverage(first, first, base, invalid, curved, ledger);
  CompareOwnerCoverage(first, first, base, next, curved, ledger, 255, 8, -1);
  const auto retry = CompareOwnerCoverage(first, first, base, next, curved, ledger);
  EXPECT_NE(retry.status, sct::NonlinearSeparationStatus::InvalidInput);
}

TEST(SelfContactOwnerRanges, RawUnsortedArraysRemainSupportedAndCheckedBorrowRejectsThem) {
  const OwnerRangeGeometry geometry;
  const auto native = DiscoverPreparedPair(geometry.first, geometry.second);
  std::vector<sct::AcceptedEventCertificate> ledger;
  for (std::size_t i = 0; i < native.count; ++i) {
    ledger.push_back(AcceptedCertificate(native.values[i]));
    ledger.back().event.source_order = i;
  }
  SortOwnerLedger(&ledger);
  std::reverse(ledger.begin(), ledger.end());
  EXPECT_THROW(sct::FinalizedCoverageLedgerTestAccess::Checked(ledger.data(), ledger.size()),
               std::invalid_argument);
  const auto raw = sct::CertifyQuadraticFacetCoverage(
      geometry.first, geometry.first, Quadratic(0), .1,
      geometry.second, geometry.second, Quadratic(0), .1, 1,
      ledger.data(), ledger.size(), 255, 8);
  EXPECT_NE(raw.status, sct::NonlinearSeparationStatus::InvalidInput);
  EXPECT_THROW(sct::FinalizedCoverageLedgerTestAccess::Checked(nullptr, 1),
               std::invalid_argument);
  EXPECT_EQ(sct::CertifyQuadraticFacetCoverage(
      geometry.first, geometry.first, Quadratic(0), .1,
      geometry.second, geometry.second, Quadratic(0), .1, 1,
      nullptr, 1, 255, 8).status, sct::NonlinearSeparationStatus::InvalidInput);
  const auto empty = sct::FinalizedCoverageLedgerTestAccess::Checked(nullptr, 0);
  const c::CurrentFixedTriangle pair[]{geometry.first, geometry.second};
  EXPECT_EQ(empty.ForPair(pair).count, 0u);
}

TEST(SelfContactOwnerRanges, DeferredExclusionCallCountsAndFailuresRemainUnchanged) {
  using namespace policy_exclusion_test;
  const SharedEdge pair;
  const auto access = sct::FinalizedCoverageLedgerTestAccess::Checked(nullptr, 0);
  for (unsigned failure = 0; failure < 3; ++failure) {
    Probe raw_probe, ranged_probe;
    raw_probe.exclusion = ranged_probe.exclusion = pair.Exclusion();
    if (failure == 1) {
      raw_probe.failure.status = ranged_probe.failure.status =
          c::SelfContactTransactionStatus::ResourceLimit;
      raw_probe.failure.message = ranged_probe.failure.message = "Frozen exclusion cap";
    }
    if (failure == 2) raw_probe.count = ranged_probe.count = 2;
    auto raw_source = raw_probe.Source();
    auto ranged_source = ranged_probe.Source();
    const auto original = pair.Run(nullptr, 0, &raw_source);
    const auto ranged = sct::CertifyQuadraticFacetPolicyCoverage(
        pair.first, pair.first, Quadratic(0), 1,
        pair.second, pair.second, Quadratic(0), 1, 1,
        access, nullptr, 0, 255, 8, &ranged_source);
    ExpectResult(original, ranged);
    EXPECT_EQ(raw_probe.calls, 1u);
    EXPECT_EQ(ranged_probe.calls, raw_probe.calls);
    EXPECT_EQ(raw_source.report.status, ranged_source.report.status);
    EXPECT_STREQ(raw_source.report.message, ranged_source.report.message);
  }
}

TEST(SelfContactOwnerRanges, OwnerCapCountsAcrossDisjointPrefixesInOriginalOrder) {
  const OwnerRangeGeometry geometry;
  const auto native = DiscoverPreparedPair(geometry.first, geometry.second);
  std::vector<sct::AcceptedEventCertificate> valid;
  for (std::size_t i = 0; i < native.count; ++i) {
    auto owner = AcceptedCertificate(native.values[i]);
    const auto result = sct::CertifyQuadraticFacetCoverage(
        geometry.first, geometry.first, Quadratic(0), .1,
        geometry.second, geometry.second, Quadratic(0), .1, 1,
        &owner, 1, 255, 8);
    if (result.status == sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage)
      valid.push_back(owner);
  }
  ASSERT_GE(valid.size(), 2u);
  std::vector<sct::AcceptedEventCertificate> ledger;
  for (unsigned i = 0; i < 65; ++i) {
    auto owner = valid[i % valid.size()];
    owner.event.classification.parent[0] = 1000 + i;
    owner.event.source_order = 65 - i;
    ledger.push_back(owner);
  }
  // Insert nonmatching full-identity prefixes between valid source vertices.
  // These retained raw-ledger rows prevent adjacent ranges from coalescing.
  for (std::size_t i = 0; i < valid.size(); ++i) {
    auto unrelated = valid[i];
    auto& key = unrelated.event.feature;
    if (key.kind != c::FixedTriangleCandidateKind::VertexFace) continue;
    ++key.vertex_face.vertex.second;
    unrelated.discovery.key = key;
    unrelated.event.source_order = 1000 + i;
    ledger.push_back(unrelated);
  }
  SortOwnerLedger(&ledger);
  const auto access = sct::FinalizedCoverageLedgerTestAccess::Checked(ledger.data(), ledger.size());
  const c::CurrentFixedTriangle pair[]{geometry.first, geometry.second};
  ASSERT_GT(access.ForPair(pair).count, 1u);
  const auto result = CompareOwnerCoverage(geometry.first, geometry.first,
      geometry.second, geometry.second, Quadratic(0), ledger);
  EXPECT_EQ(result.status, sct::NonlinearSeparationStatus::OwnerAmbiguity);
}

TEST(SelfContactOwnerRanges, EveryVertexSourceIdentityFieldRemainsSignificant) {
  const OwnerRangeGeometry geometry;
  const auto native = DiscoverPreparedPair(geometry.first, geometry.second);
  sct::AcceptedEventCertificate base;
  bool found = false;
  for (std::size_t i = 0; i < native.count; ++i) {
    if (native.values[i].key.kind != c::FixedTriangleCandidateKind::VertexFace) continue;
    auto owner = AcceptedCertificate(native.values[i]);
    const auto result = sct::CertifyQuadraticFacetCoverage(
        geometry.first, geometry.first, Quadratic(0), .1,
        geometry.second, geometry.second, Quadratic(0), .1, 1,
        &owner, 1, 255, 8);
    if (result.status == sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage) {
      base = owner; found = true; break;
    }
  }
  ASSERT_TRUE(found);
  for (unsigned mutation = 0; mutation < 9; ++mutation) {
    auto owner = base;
    auto& key = owner.event.feature.vertex_face.vertex;
    if (mutation == 0) ++key.source_instance_id;
    if (mutation == 1) key.kind = c::FacetVertexKind::SourceEdge;
    if (mutation == 2) key.first += 1000;
    if (mutation == 3) ++key.second;
    if (mutation == 4) ++key.numerator;
    if (mutation == 5) ++key.denominator;
    if (mutation == 6) ++key.level;
    if (mutation == 7) ++key.grid_i;
    if (mutation == 8) ++key.grid_j;
    owner.discovery.key = owner.event.feature;
    SCOPED_TRACE(mutation);
    const auto result = CompareOwnerCoverage(geometry.first, geometry.first,
        geometry.second, geometry.second, Quadratic(0), {owner});
    EXPECT_NE(result.status, sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage);
  }
}

TEST(SelfContactOwnerRanges, SelectedCertificateRetainsNonzeroAbsoluteOrdinal) {
  const OwnerRangeGeometry geometry;
  const auto native = DiscoverPreparedPair(geometry.first, geometry.second);
  sct::AcceptedEventCertificate covering;
  bool found = false;
  for (std::size_t i = 0; i < native.count; ++i) {
    if (native.values[i].key.kind != c::FixedTriangleCandidateKind::VertexFace) continue;
    auto owner = AcceptedCertificate(native.values[i]);
    const auto result = sct::CertifyQuadraticFacetCoverage(
        geometry.first, geometry.first, Quadratic(0), .1,
        geometry.second, geometry.second, Quadratic(0), .1, 1,
        &owner, 1, 255, 8);
    if (result.status == sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage) {
      covering = owner; found = true; break;
    }
  }
  ASSERT_TRUE(found);
  covering.event.source_order = 23;
  auto unrelated = covering;
  unrelated.event.feature.vertex_face.vertex.source_instance_id = 0;
  unrelated.discovery.key = unrelated.event.feature;
  unrelated.event.source_order = 0;
  std::vector<sct::AcceptedEventCertificate> ledger{covering, unrelated};
  SortOwnerLedger(&ledger);
  const auto original = sct::CertifyQuadraticFacetCoverage(
      geometry.first, geometry.first, Quadratic(0), .1,
      geometry.second, geometry.second, Quadratic(0), .1, 1,
      ledger.data(), ledger.size(), 255, 8);
  ASSERT_EQ(original.status, sct::NonlinearSeparationStatus::CertifiedAcceptedCoverage);
  ASSERT_GT(original.accepted_certificate, 0u);
  ASSERT_EQ(original.accepted_source_order, 23u);
  const auto ranged = CompareOwnerCoverage(geometry.first, geometry.first,
      geometry.second, geometry.second, Quadratic(0), ledger);
  policy_exclusion_test::ExpectResult(original, ranged);
}
