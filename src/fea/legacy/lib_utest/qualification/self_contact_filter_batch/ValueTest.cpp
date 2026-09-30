// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Corpus.h"
#include "lib_src/collision/self_contact_filters/Prism.h"
#include "lib_src/collision/self_contact_filters/Storage.h"
#include <gtest/gtest.h>

namespace {
namespace c = tlfea::contact;
namespace f = c::self_contact_filters;

TEST(SelfContactFilterBatchValues, CompactGeometryPreservesEveryPublicCertificateField) {
  const auto corpus = filter_batch_test::MakeCorpus();
  for (const auto pair : corpus.pairs) {
    const auto& a = corpus.properties[pair.first];
    const auto& b = corpus.properties[pair.second];
    const auto accepted = f::detail::ClassifyAcceptedFacetPairImpl(
        corpus.accepted[pair.first], a.half_thickness, a.complete_rigid_group,
        corpus.accepted[pair.second], b.half_thickness, b.complete_rigid_group);
    const auto expected = filter_batch_test::Reference(corpus, pair, true);
    EXPECT_EQ(accepted.status, expected.status);
    EXPECT_EQ(accepted.category, expected.category);
    for (unsigned limit = 0; limit < 4; ++limit) {
      const auto axes = static_cast<c::SelfContactFacetPrismAxisLimit>(limit);
      f::PairResult actual;
      bool valid = false;
      actual.separated = f::detail::CertifiedLinearFacetPrismSeparationImpl<true, false>(
          corpus.accepted[pair.first], corpus.prepared[pair.first], a.half_thickness,
          corpus.accepted[pair.second], corpus.prepared[pair.second], b.half_thickness,
          axes, &actual.axis, &valid, nullptr);
      actual.status = valid ? c::SelfContactFacetFilterStatus::Ok : c::SelfContactFacetFilterStatus::InvalidInput;
      EXPECT_TRUE(filter_batch_test::Same(actual, filter_batch_test::Reference(corpus, pair, false, axes)));
    }
  }
}

TEST(SelfContactFilterBatchValues, ExactStorageForecastIncludesBothEndpointsAndStartupFootprint) {
  f::Limits limits;
  limits.max_facets = 7; limits.max_pairs = 11;
  constexpr std::size_t owner_bytes = 137;
  f::Layout layout;
  ASSERT_EQ(f::MakeLayout(limits, owner_bytes, layout).status, f::Status::Ok);
  const auto device = 7 * (2 * sizeof(f::TriangleGeometry) + sizeof(f::FacetProperties)) +
                      11 * (sizeof(c::FixedTrianglePair) + sizeof(f::PairResult));
  EXPECT_EQ(layout.forecast.device_bytes, device);
  EXPECT_EQ(layout.forecast.owned_host_bytes, owner_bytes + 11 * sizeof(f::PairResult));
  EXPECT_GT(layout.forecast.startup_host_bytes, layout.forecast.owned_host_bytes);
  EXPECT_EQ(layout.forecast.device_allocations, 1u);
  limits.max_device_bytes = device;
  limits.max_host_bytes = layout.forecast.startup_host_bytes;
  f::Layout exact;
  ASSERT_EQ(f::MakeLayout(limits, owner_bytes, exact).status, f::Status::Ok);
  const auto prior = exact.forecast;
  --limits.max_device_bytes;
  EXPECT_EQ(f::MakeLayout(limits, owner_bytes, exact).status, f::Status::ResourceLimit);
  EXPECT_EQ(exact.forecast.device_bytes, prior.device_bytes);
  ++limits.max_device_bytes; --limits.max_host_bytes;
  EXPECT_EQ(f::MakeLayout(limits, owner_bytes, exact).status, f::Status::ResourceLimit);
  EXPECT_EQ(exact.forecast.startup_host_bytes, prior.startup_host_bytes);
}

TEST(SelfContactFilterBatchValues, InvalidAndOverflowingCapacityNeverPublishesLayout) {
  f::Layout layout;
  for (unsigned mutation = 0; mutation < 5; ++mutation) {
    f::Limits limits;
    if (mutation == 0) limits.max_facets = 0;
    if (mutation == 1) limits.max_pairs = 0;
    if (mutation == 2) limits.max_pairs = SIZE_MAX;
    if (mutation == 3) limits.max_device_bytes = 0;
    if (mutation == 4) limits.max_host_bytes = 0;
    EXPECT_EQ(f::MakeLayout(limits, 256, layout).status, f::Status::InvalidInput);
    EXPECT_EQ(layout.forecast.device_bytes, 0u);
  }
}
}  // namespace
