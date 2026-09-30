// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <type_traits>

namespace extended_solid_test {
namespace {
void Empty(const fe::SolidNodeContributions& result) {
  EXPECT_FALSE(result.prepared());
  EXPECT_EQ(result.domain(), nullptr);
  EXPECT_EQ(result.parents().size(), 0u);
  EXPECT_EQ(result.startup_payload_bytes(), sizeof(result));
  EXPECT_EQ(result.owned_payload_bytes(), sizeof(result));
}
template<class Reference>
void CheckRange(const Fixture& fixture, const fe::NodalNodeDomain& domain,
    const Reference* fe::SolidCoefficientInput::* pointer,
    std::size_t fe::SolidCoefficientInput::* count) {
  fe::SolidNodeContributions result;
  auto input = fixture.Input();
  input.*pointer = nullptr;
  EXPECT_EQ(result.Initialize(domain, input).status, Status::InvalidInput);
  input = fixture.Input();
  input.*count = 0;
  EXPECT_EQ(result.Initialize(domain, input).status, Status::InvalidInput);
  input = fixture.Input();
  input.*count = SIZE_MAX;
  EXPECT_EQ(result.Initialize(domain, input).status, Status::ResourceLimit);
  input = fixture.Input();
  input.*pointer = reinterpret_cast<const Reference*>(reinterpret_cast<std::uintptr_t>(input.*pointer)+1);
  EXPECT_EQ(result.Initialize(domain, input).status, Status::InvalidInput);
  input = fixture.Input();
  constexpr auto last_aligned = UINTPTR_MAX-(alignof(Reference)-1);
  input.*pointer = reinterpret_cast<const Reference*>(last_aligned);
  EXPECT_EQ(result.Initialize(domain, input).status, Status::InvalidInput);
  Empty(result);
  ASSERT_TRUE(result.Initialize(domain, fixture.Input()));
}
}
TEST(ExtendedSolidCoefficients, AllFivePointerCountPairsRejectBeforeAccessAndPermitRetry) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  using I = fe::SolidCoefficientInput;
  CheckRange(fixture, domain, &I::solid18, &I::solid18_count);
  CheckRange(fixture, domain, &I::solid24, &I::solid24_count);
  CheckRange(fixture, domain, &I::solid6z, &I::solid6z_count);
  CheckRange(fixture, domain, &I::law44, &I::law44_count);
  CheckRange(fixture, domain, &I::law90, &I::law90_count);
}
TEST(ExtendedSolidCoefficients, CombinedCountsDomainAndInclusiveBytesHaveExactCaps) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  fe::SolidNodeContributions known, result;
  ASSERT_TRUE(known.Initialize(domain, fixture.Input()));
  fe::SolidCoefficientLimits limits;
  limits.max_parents = 4;
  EXPECT_EQ(result.Initialize(domain, fixture.Input(), limits).status, Status::ResourceLimit);
  limits.max_parents = 5;
  limits.max_nodes = domain.node_count()-1;
  EXPECT_EQ(result.Initialize(domain, fixture.Input(), limits).status, Status::ResourceLimit);
  limits.max_nodes = domain.node_count();
  limits.max_host_bytes = known.startup_payload_bytes()-1;
  EXPECT_EQ(result.Initialize(domain, fixture.Input(), limits).status, Status::ResourceLimit);
  Empty(result);
  ++limits.max_host_bytes;
  ASSERT_TRUE(result.Initialize(domain, fixture.Input(), limits));
  EXPECT_TRUE(result.Matches(known));
  EXPECT_GT(result.startup_payload_bytes(), result.owned_payload_bytes());
  EXPECT_GT(result.owned_payload_bytes(), domain.owned_payload_bytes());
  EXPECT_EQ(result.parent_count(static_cast<Family>(99)), 0u);
}
TEST(ExtendedSolidCoefficients, UnpreparedLastFamilyAndWrongSourceKeepEmptyUntilValidCommit) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  fe::SolidNodeContributions result;
  auto input = fixture.Input();
  input.source_instance_id = 2;
  EXPECT_EQ(result.Initialize(domain, input).status, Status::InvalidInput);
  fe::solid18::total_strain::Reference unprepared;
  input = fixture.Input();
  input.law90 = &unprepared;
  const auto report = result.Initialize(domain, input);
  EXPECT_EQ(report.status, Status::InvalidInput);
  EXPECT_EQ(report.node, 4u);
  Empty(result);
  ASSERT_TRUE(result.Initialize(domain, fixture.Input()));
  const auto* retained_rows = result.parents().data();
  EXPECT_EQ(result.Initialize(domain, input).status, Status::AlreadyInitialized);
  EXPECT_EQ(result.parents().data(), retained_rows);
  EXPECT_EQ(result.parents()[4].source_element_id, fixture.foam_input.source_element_id);
}
} // namespace extended_solid_test
