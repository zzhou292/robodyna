#include "modelio/source_assembly/NativeCoordinates.h"
#include "modelio/tied_shell/tests/TinyFixture.h"
#include "modelio/tied_shell/search_geometry/Internal.h"
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::coated::test {
namespace tied = modelio::tied_shell;
TEST(V5NativeCoordinates, ArbitraryRequestedOrderRetainsOriginalDecimalAndZeroBits) {
    tied::test::TinyFixture fixture(false, true);
    const auto ids = tied::search_detail::Decode<std::uint64_t>(fixture.canonical, "node_ids");
    const auto result = modelio::source_nodes::ReadNativeCoordinates(fixture.canonical, ids, {11,1,0}, fixture.member);
    ASSERT_EQ(result.positions.size(), 3u);
    EXPECT_EQ(result.positions[0][0], 1);
    EXPECT_EQ(result.positions[1][0], 15.7);
    EXPECT_NE(result.positions[1][0], (15.7*.001)/.001);
    EXPECT_TRUE(std::signbit(result.positions[2][0]));
    EXPECT_EQ(result.roundtrip_changed_components, 1u);
    EXPECT_GT(modelio::source_nodes::NativeCoordinateBytes(fixture.canonical, 3), result.positions.size()*sizeof(result.positions[0]));
}
TEST(V5NativeCoordinates, DuplicateRangeIdentityAndSourceCodeFailuresRejectThenRetry) {
    tied::test::TinyFixture fixture(false, true);
    auto ids = tied::search_detail::Decode<std::uint64_t>(fixture.canonical, "node_ids");
    EXPECT_THROW(modelio::source_nodes::ReadNativeCoordinates(fixture.canonical, ids, {0,0}, fixture.member), std::exception);
    EXPECT_THROW(modelio::source_nodes::ReadNativeCoordinates(fixture.canonical, ids, {99}, fixture.member), std::exception);
    const auto first = ids[0]; ids[0] = 999;
    EXPECT_THROW(modelio::source_nodes::ReadNativeCoordinates(fixture.canonical, ids, {0}, fixture.member), std::exception);
    ids[0] = first;
    EXPECT_NO_THROW(modelio::source_nodes::ReadNativeCoordinates(fixture.canonical, ids, {0}, fixture.member));
    EXPECT_THROW(modelio::source_nodes::NativeCoordinateBytes(fixture.canonical, SIZE_MAX), std::exception);
}
} // namespace crash::cases::vehicle_self_contact::native::coated::test
