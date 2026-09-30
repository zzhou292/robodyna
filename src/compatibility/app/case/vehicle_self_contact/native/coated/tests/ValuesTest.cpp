#include "Fixture.h"
#include "../Internal.h"
#include <climits>
#include <limits>
namespace crash::cases::vehicle_self_contact::native::coated::test {
TEST(V5CoatedValues, PentaReaderRetainsItsEightPreInitiaSlotsForBothDeclaredOrientations) {
    auto input = Cube();
    const std::array<std::uint32_t, 8> first{0,1,2,4,5,6,UINT32_MAX,UINT32_MAX};
    const std::array<std::uint32_t, 8> reversed{4,5,6,0,1,2,UINT32_MAX,UINT32_MAX};
    EXPECT_EQ(ReaderSlots(ReaderKind::DeclaredPenta6, first, input.nodes),
              (std::array<std::uint32_t,8>{0,1,2,0,4,5,6,4}));
    EXPECT_EQ(ReaderSlots(ReaderKind::DeclaredPenta6, reversed, input.nodes),
              (std::array<std::uint32_t,8>{4,5,6,4,0,1,2,0}));
    // Geometry mechanics may orient these opposite inputs to the same active
    // six slots. Reader output intentionally precedes that different phase.
}
TEST(V5CoatedValues, CompleteMembershipCountsPhysicalAndContactScopesWithoutPruning) {
    auto input = Cube();
    Shell other = input.shells[0];
    other.primary.source_id = 101; other.physical_parent = 1; other.contact_selected = false;
    input.shells.push_back(other);
    const auto result = Classify(input);
    ASSERT_TRUE(result.complete);
    EXPECT_EQ(result.physical.shells, 2u); EXPECT_EQ(result.contact.shells, 1u);
    EXPECT_EQ(result.physical.matches, 2u); EXPECT_EQ(result.contact.matches, 1u);
    EXPECT_EQ(result.physical.unique, 2u); EXPECT_EQ(result.physical.multiple, 0u);
    const auto order = SurfaceOrder(input, result);
    EXPECT_EQ(order.primary_to_physical, (std::vector<std::uint32_t>{0}));
    EXPECT_EQ(order.physical_to_primary, (std::vector<std::uint32_t>{0,UINT32_MAX}));
    EXPECT_EQ(input.shells.size(), 2u); EXPECT_EQ(input.solids.size(), 1u);
}
TEST(V5CoatedValues, NodeIdentityAndAmbiguousFullIncidenceRemainExplicit) {
    auto input = Cube();
    // Coincident source nodes remain different identities and do not match.
    for (unsigned i = 4; i < 8; ++i) {
        auto node = input.nodes[i]; node.source_id += 100; node.canonical_row += 100;
        input.nodes.push_back(node);
    }
    input.shells[0].primary.nodes[0] = 8; input.shells[0].primary.nodes[1] = 9;
    input.shells[0].primary.nodes[2] = 10; input.shells[0].primary.nodes[3] = 11;
    auto result = Classify(input);
    EXPECT_TRUE(result.complete); EXPECT_EQ(result.physical.unmatched, 1u);
    EXPECT_EQ(result.roles[0].state, RoleState::Ordinary);
    input = Cube();
    auto second = input.solids[0]; second.source_id = 501;
    input.solids.push_back(second);
    result = Classify(input);
    EXPECT_FALSE(result.complete); EXPECT_EQ(result.physical.multiple, 1u);
    EXPECT_EQ(result.roles[0].matches, 2u); EXPECT_EQ(result.roles[0].state, RoleState::Unresolved);
    EXPECT_EQ(result.first_unready_shell, 0u);
    EXPECT_THROW(SurfaceOrder(input, result), std::exception);
    EXPECT_EQ(input.solids.size(), 2u);
}
TEST(V5CoatedValues, UnsignedRoleWordSortAndNativeIdentityBoundsAreLiteral) {
    auto input = Cube();
    auto second = input.shells[0]; second.primary.source_id = 101; second.physical_parent = 1;
    input.shells.push_back(second);
    Classification values;
    values.complete = true; values.contact_complete = true; values.physical.shells = values.contact.shells = 2;
    values.roles.resize(2);
    // Explicit ordering-only numerical packet. This does not pretend that the
    // two supplied signs were produced by a physical membership classification.
    values.roles[0].state = RoleState::CoatingReversed;
    values.roles[1].state = RoleState::CoatingForward;
    EXPECT_EQ(SurfaceOrder(input, values).primary_to_physical, (std::vector<std::uint32_t>{1,0}));
    values.roles[1].state = RoleState::CoatingReversed;
    EXPECT_THROW(SurfaceOrder(input, values), std::exception);
    input = Cube(); values = Classify(input);
    input.nodes.back().source_id = std::uint64_t(INT_MAX) + 1;
    EXPECT_THROW(SurfaceOrder(input, values), std::exception);
    input.nodes.back().source_id = input.nodes[0].source_id;
    EXPECT_THROW(SurfaceOrder(input, values), std::exception);
}
TEST(V5CoatedValues, InvalidReaderPhaseSlotsAndSourceGeometryNeverBecomeRoles) {
    auto input = Cube();
    const std::array<std::uint32_t,8> declared{0,1,2,3,4,5,6,7};
    input.nodes[7].native_position.x = std::numeric_limits<double>::infinity();
    EXPECT_THROW(ReaderSlots(ReaderKind::Hex8, declared, input.nodes), std::exception);
    input = Cube(); input.solids[0].phase = static_cast<PacketPhase>(99);
    EXPECT_THROW(Classify(input), std::exception);
    input = Cube(); input.shells[0].primary.nodes[2] = input.shells[0].primary.nodes[1];
    EXPECT_THROW(Classify(input), std::exception);
    input = Cube(); input.solids[0].nodes[7] = UINT32_MAX;
    EXPECT_THROW(Classify(input), std::exception);
    const auto retry = Classify(Cube());
    EXPECT_TRUE(retry.complete); EXPECT_EQ(retry.physical.unique, 1u);
}
TEST(V5CoatedValues, NonContactAmbiguityRemainsReportedWithoutBlockingSelectedOrder) {
    auto input = Cube();
    auto noncontact = input.shells[0];
    noncontact.primary.source_id = 101; noncontact.physical_parent = 1; noncontact.contact_selected = false;
    // Separate coincident geometry retains distinct source identities. Only this
    // non-contact shell belongs to two retained solids; the selected row is unique.
    for (auto node : Cube().nodes) {
        node.source_id += 10; node.canonical_row += 8; input.nodes.push_back(node);
    }
    for (auto& node : noncontact.primary.nodes) node += 8;
    input.shells.push_back(noncontact);
    auto second = input.solids[0]; second.source_id = 501;
    for (auto& node : second.nodes) node += 8;
    input.solids.push_back(second); second.source_id = 502; input.solids.push_back(second);
    const auto result = Classify(input);
    EXPECT_FALSE(result.complete); EXPECT_TRUE(result.contact_complete);
    EXPECT_EQ(result.physical.multiple, 1u); EXPECT_EQ(result.contact.multiple, 0u);
    EXPECT_EQ(result.roles[1].state, RoleState::Unresolved);
    EXPECT_EQ(result.first_unready_shell, 1u); EXPECT_EQ(result.first_unready_contact_shell, SIZE_MAX);
    EXPECT_EQ(SurfaceOrder(input, result).primary_to_physical, (std::vector<std::uint32_t>{0}));
    EXPECT_EQ(input.shells.size(), 2u); EXPECT_EQ(input.solids.size(), 3u);
}
TEST(V5CoatedValues, TypedDigestBindsRolesReaderPhaseOriginalSourceAndOrder) {
    auto input = Cube();
    const auto classified = Classify(input); const auto order = SurfaceOrder(input, classified);
    const auto digest = detail::InputDigest(input, classified, &order, {}, std::string(64,'a'), 1u<<20);
    auto copy = input;
    EXPECT_EQ(detail::InputDigest(copy, classified, &order, {}, std::string(64,'a'), 1u<<20).sha256, digest.sha256);
    copy.solids[0].nodes[3] = 4;
    EXPECT_NE(detail::InputDigest(copy, classified, &order, {}, std::string(64,'a'), 1u<<20).sha256, digest.sha256);
    EXPECT_NE(detail::InputDigest(input, classified, &order, {}, std::string(64,'b'), 1u<<20).sha256, digest.sha256);
    auto roles = classified; roles.roles[0].state = RoleState::CoatingReversed;
    if (roles.roles[0].state == classified.roles[0].state) roles.roles[0].state = RoleState::CoatingForward;
    EXPECT_NE(detail::InputDigest(input, roles, &order, {}, std::string(64,'a'), 1u<<20).sha256, digest.sha256);
    // Changed digest inputs are not a source-admission success claim.
    EXPECT_THROW(detail::InputDigest(input, classified, &order, {}, std::string(64,'a'), 1), std::exception);
}
} // namespace crash::cases::vehicle_self_contact::native::coated::test
