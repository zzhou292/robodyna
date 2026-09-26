// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace nodal_correction_test {
namespace {
struct CertificateFixture {
  tl::util::HostArena scratch;
  c::Forecast forecast;
  c::OrderCertificate certificate{91, 92, 93, 94};
  c::OrderInput Input(const Fixture& f) const {
    return {f.solids.empty() ? nullptr : f.solids.data(), f.solids.size(), f.original.size()};
  }
  void Prepare(const Fixture& f) {
    if (c::PreflightOrderCertificate(Input(f), {}, forecast).status != c::Status::Ok ||
        !scratch.Initialize(forecast.scratch_bytes)) throw std::runtime_error("Certificate fixture admission");
  }
  c::OrderReport Certify(const Fixture& f) {
    return c::CertifyOrder(Input(f), {}, scratch.data(), scratch.bytes(), &certificate);
  }
};
void SameCertificate(const c::OrderCertificate& a, const c::OrderCertificate& b) {
  EXPECT_EQ(a.node_count, b.node_count);
  EXPECT_EQ(a.solid_count, b.solid_count);
  EXPECT_EQ(a.controlled_solids, b.controlled_solids);
  EXPECT_EQ(a.affected_nodes, b.affected_nodes);
}
}

TEST(NodalCorrectionOrder, SharedFactorMatchesNativeFloorAndSignedZeroWithoutConsumingInactiveFields) {
  const double floor = 1. / 1e20;
  for (double bulk : {0., -0., std::nextafter(floor, 0.), floor, std::nextafter(floor, 1.), 7.}) {
    for (double controlled : {0., -0., 2e-20, 14.}) {
      Fixture f;
      f.original[0] = 1.;
      f.solids = {Solid(bulk, controlled)};
      c::FactorResult result{false, 19.};
      ASSERT_EQ(c::EvaluateFactor(f.solids[0], &result), c::Status::Ok);
      ASSERT_TRUE(result.active);
      EXPECT_TRUE(tl::math::SameScalarBits(result.value, Oracle(f.Input())[0]));
    }
  }
  auto inactive = Solid(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), 2);
  inactive.nodes[0] = UINT32_MAX;
  c::FactorResult result{true, 91.};
  ASSERT_EQ(c::EvaluateFactor(inactive, &result), c::Status::Ok);
  EXPECT_FALSE(result.active);
  EXPECT_EQ(result.value, 0.); // API metadata only: no native factor was computed.
  auto overflow = Solid(floor, std::numeric_limits<double>::max());
  result = {true, 91.};
  EXPECT_EQ(c::EvaluateFactor(overflow, &result), c::Status::NonfiniteResult);
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.value, 91.);
}

TEST(NodalCorrectionOrder, DifferentMaterialOperandsWithEqualNodeFactorsPermitEveryPermutation) {
  const auto a = Solid(2., 6.);
  const auto b = Solid(4., 12.);
  auto different_node = Solid(1., 7.);
  for (auto& node : different_node.nodes) node = 8;
  const std::array<c::Solid, 3> rows{a, b, different_node};
  std::array<unsigned, 3> order{0, 1, 2};
  std::vector<double> first;
  do {
    Fixture f;
    for (auto i : order) f.solids.push_back(rows[i]);
    f.secondaries = {9, 9, UINT32_MAX};
    CertificateFixture proof;
    proof.Prepare(f);
    ASSERT_EQ(proof.Certify(f).status, c::OrderStatus::Ok);
    EXPECT_EQ(proof.certificate.controlled_solids, 3u);
    EXPECT_EQ(proof.certificate.affected_nodes, 8u);
    ASSERT_EQ(f.Prepare().status, c::Status::Ok);
    ASSERT_EQ(f.Run().status, c::Status::Ok);
    Same(f.output, Oracle(f.Input()));
    if (first.empty()) first = f.output;
    else Same(f.output, first);
  } while (std::next_permutation(order.begin(), order.end()));
}

TEST(NodalCorrectionOrder, ConflictingSharedFactorRequiresNativeOrderAndPreservesCertificate) {
  Fixture f;
  f.solids = {Solid(2., 6.), Solid(2., 8.)};
  CertificateFixture proof;
  proof.Prepare(f);
  const auto prior = proof.certificate;
  const auto report = proof.Certify(f);
  EXPECT_EQ(report.status, c::OrderStatus::NeedsNativeStorageOrder);
  EXPECT_EQ(report.first_solid, 0u);
  EXPECT_EQ(report.conflicting_solid, 1u);
  EXPECT_EQ(report.node, 0u);
  SameCertificate(proof.certificate, prior);
  const auto forward = Oracle(f.Input());
  std::reverse(f.solids.begin(), f.solids.end());
  const auto reverse = Oracle(f.Input());
  EXPECT_NE(forward[0], reverse[0]);
  f.solids[1] = f.solids[0];
  EXPECT_EQ(proof.Certify(f).status, c::OrderStatus::Ok);
}

TEST(NodalCorrectionOrder, EqualNumericZerosWithDifferentBitsDoNotAuthorizePermutation) {
  Fixture f;
  f.solids = {Solid(1., 0.), Solid(1., -0.)};
  CertificateFixture proof;
  proof.Prepare(f);
  EXPECT_EQ(proof.Certify(f).status, c::OrderStatus::NeedsNativeStorageOrder);
  const auto positive = Oracle(f.Input());
  std::reverse(f.solids.begin(), f.solids.end());
  const auto negative = Oracle(f.Input());
  EXPECT_FALSE(tl::math::SameScalarBits(positive[0], negative[0]));
}

TEST(NodalCorrectionOrder, EmptyAndDisabledRowsLeaveUnusedOperandsUnread) {
  for (bool disabled : {false, true}) {
    Fixture f;
    if (disabled) {
      f.solids = {Solid(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), 0)};
      for (auto& node : f.solids[0].nodes) node = UINT32_MAX;
    }
    CertificateFixture proof;
    proof.Prepare(f);
    ASSERT_EQ(proof.Certify(f).status, c::OrderStatus::Ok);
    EXPECT_EQ(proof.certificate.controlled_solids, 0u);
    EXPECT_EQ(proof.certificate.affected_nodes, 0u);
  }
}

TEST(NodalCorrectionOrder, CountCapAliasAndLateInvalidInputCannotPublish) {
  Fixture f;
  f.solids = {Solid(2., 6.), Solid(4., 12.)};
  CertificateFixture proof;
  proof.Prepare(f);
  const auto prior = proof.certificate;
  auto limits = c::Limits{};
  limits.scratch_bytes = proof.forecast.scratch_bytes;
  c::Forecast forecast;
  EXPECT_EQ(c::PreflightOrderCertificate(proof.Input(f), limits, forecast).status, c::Status::Ok);
  --limits.scratch_bytes;
  EXPECT_EQ(c::PreflightOrderCertificate(proof.Input(f), limits, forecast).status, c::Status::ResourceLimit);
  auto bad = proof.Input(f);
  bad.solids = reinterpret_cast<const c::Solid*>(1);
  bad.solid_count = c::Limits{}.solids + 1;
  EXPECT_EQ(c::PreflightOrderCertificate(bad, {}, forecast).status, c::Status::ResourceLimit);
  f.solids[1].nodes[7] = f.original.size();
  EXPECT_EQ(proof.Certify(f).status, c::OrderStatus::InvalidInput);
  SameCertificate(proof.certificate, prior);
  f.solids[1].nodes[7] = 6;
  EXPECT_EQ(c::CertifyOrder(proof.Input(f), {}, proof.scratch.data(), proof.scratch.bytes()-1,
      &proof.certificate).status, c::OrderStatus::InvalidInput);
  auto* alias = ::new (proof.scratch.data()) c::OrderCertificate{81, 82, 83, 84};
  const auto saved = *alias;
  EXPECT_EQ(c::CertifyOrder(proof.Input(f), {}, proof.scratch.data(), proof.scratch.bytes(), alias).status,
      c::OrderStatus::InvalidInput);
  SameCertificate(*alias, saved);
  EXPECT_EQ(proof.Certify(f).status, c::OrderStatus::Ok);
}
} // namespace nodal_correction_test
