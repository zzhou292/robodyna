#include "Fixture.h"
#include "NativeOracle.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::coated::test {
namespace {
int RawRole(n::ShellLayout layout, RoleState state) {
    const int ordinary = layout == n::ShellLayout::Triangle3 ? 7 : 3;
    if (state == RoleState::Ordinary) return ordinary;
    if (state == RoleState::CoatingForward) return ordinary + 1;
    EXPECT_EQ(state, RoleState::CoatingReversed);
    return -(ordinary + 1);
}
void SameNativeRoles(const Inputs& actual_input, const Inputs& native_input) {
    const auto actual = Classify(actual_input);
    const auto expected = NativeRoles(native_input);
    ASSERT_TRUE(actual.complete);
    ASSERT_EQ(actual.roles.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(RawRole(actual_input.shells[i].primary.layout, actual.roles[i].state), expected[i]);
    }
    EXPECT_EQ(SurfaceOrder(actual_input, actual).primary_to_physical, NativeOrder(native_input, expected));
}
}
TEST(V5CoatedNative, CompleteReaderPacketsMatchOriginalH8AndPentaBranches) {
    for (ReaderKind kind : {ReaderKind::Hex8, ReaderKind::DeclaredPenta6}) {
        for (bool reversed : {false, true}) for (unsigned change = 0; change < 5; ++change) {
            SCOPED_TRACE(int(kind));
            SCOPED_TRACE(reversed);
            SCOPED_TRACE(change);
            auto input = Cube();
            std::array<std::uint32_t,8> declared = kind == ReaderKind::Hex8 ?
                std::array<std::uint32_t,8>{0,1,2,3,4,5,6,7} :
                std::array<std::uint32_t,8>{0,1,2,4,5,6,UINT32_MAX,UINT32_MAX};
            const unsigned half = kind == ReaderKind::Hex8 ? 4 : 3;
            if (reversed) for (unsigned i = 0; i < half; ++i) std::swap(declared[i], declared[half+i]);
            for (auto& node : input.nodes) {
                auto& p = node.native_position;
                if (change == 0) { const auto x = p.x; p.x = .8*x-.6*p.z; p.z = .6*x+.8*p.z; }
                if (change == 1) p.z += .0125*p.x*p.y;
                if (change == 2) { p.x *= 1e-8; p.y *= 1e-8; p.z *= 1e-8; }
                if (change == 3) { p.x += 105.25; p.y -= 36.5; p.z += 9.125; }
                if (change == 4) { if (p.x == 0) p.x = -0.; if (p.y == 0) p.y = -0.; }
            }
            EXPECT_EQ(ReaderSlots(kind, declared, input.nodes), NativeReader(kind, declared, input.nodes));
        }
    }
    auto input = Cube();
    const std::array<std::uint32_t,8> repeated{0,1,2,3,4,4,6,6};
    EXPECT_EQ(ReaderSlots(ReaderKind::Hex8, repeated, input.nodes), NativeReader(ReaderKind::Hex8, repeated, input.nodes));
}
TEST(V5CoatedNative, ReaderPhaseFeedsFullIndependentInclusionAndSourceOrdering) {
    for (bool reversed : {false, true}) {
        SCOPED_TRACE(reversed);
        auto actual = Cube();
        auto native = actual;
        const std::array<std::uint32_t,8> first{0,1,2,3,4,5,6,7};
        actual.solids[0].nodes = ReaderSlots(ReaderKind::Hex8, first, actual.nodes);
        native.solids[0].nodes = NativeReader(ReaderKind::Hex8, first, native.nodes);
        const auto second_nodes = Cube().nodes;
        for (auto node : second_nodes) {
            node.source_id += 10; node.canonical_row += 8; node.native_position.x += 12;
            actual.nodes.push_back(node); native.nodes.push_back(node);
        }
        const std::array<std::uint32_t,8> penta = reversed ?
            std::array<std::uint32_t,8>{12,13,14,8,9,10,UINT32_MAX,UINT32_MAX} :
            std::array<std::uint32_t,8>{8,9,10,12,13,14,UINT32_MAX,UINT32_MAX};
        Solid second; second.source_id = 501; second.part_id = 31; second.kind = ReaderKind::DeclaredPenta6;
        second.nodes = ReaderSlots(second.kind, penta, actual.nodes); actual.solids.push_back(second);
        second.nodes = NativeReader(second.kind, penta, native.nodes); native.solids.push_back(second);
        Shell triangle;
        triangle.primary = {101,n::ShellLayout::Triangle3,{12,13,14,14}};
        triangle.part_id = 21; triangle.physical_parent = 1; triangle.contact_selected = true;
        actual.shells.push_back(triangle); native.shells.push_back(triangle);
        auto extra = actual.shells[0]; extra.primary.source_id = 102; extra.physical_parent = 2; extra.contact_selected = false;
        actual.shells.push_back(extra); native.shells.push_back(extra);
        SameNativeRoles(actual, native);
        // Directed reversal changes the source branch using the same packet,
        // not a production-provided sign or an oracle-provided role seed.
        for (auto* value : {&actual, &native}) {
            std::swap(value->shells[0].primary.nodes[0], value->shells[0].primary.nodes[1]);
            std::swap(value->shells[0].primary.nodes[2], value->shells[0].primary.nodes[3]);
            std::swap(value->shells[1].primary.nodes[0], value->shells[1].primary.nodes[1]);
        }
        SameNativeRoles(actual, native);
    }
}
TEST(V5CoatedNative, UnmatchedIdentityAndAmbiguousMembershipKeepTheirDistinctContracts) {
    auto input = Cube();
    input.nodes.push_back({99,99,input.nodes[4].native_position});
    input.shells[0].primary.nodes[0] = 8;
    const auto roles = Classify(input);
    ASSERT_TRUE(roles.complete);
    EXPECT_EQ(roles.physical.unmatched, 1u);
    EXPECT_EQ(NativeRoles(input), (std::vector<int>{3}));
    input = Cube();
    auto duplicate = input.solids[0]; duplicate.source_id = 501; input.solids.push_back(duplicate);
    const auto ambiguous = Classify(input);
    EXPECT_FALSE(ambiguous.complete); EXPECT_EQ(ambiguous.roles[0].matches, 2u);
    // Original IN24 chooses its first native solid. Without authentic incidence
    // ordering this source profile deliberately publishes no role; no deletion.
    ASSERT_EQ(NativeRoles(input).size(), 1u);
    EXPECT_EQ(ambiguous.roles[0].state, RoleState::Unresolved);
}
TEST(V5CoatedNative, OriginalUnsignedSixWordSortChecksNegativeRoleAndStableTies) {
    auto input = Cube();
    auto duplicate = input.shells[0]; duplicate.primary.source_id = 101; duplicate.physical_parent = 1;
    input.shells.push_back(duplicate);
    Classification classified; classified.complete = true; classified.contact_complete = true;
    classified.physical.shells = classified.contact.shells = 2;
    classified.roles.resize(2); classified.roles[0].state = RoleState::CoatingReversed;
    classified.roles[1].state = RoleState::CoatingForward;
    EXPECT_EQ(SurfaceOrder(input, classified).primary_to_physical, NativeOrder(input, {-4,4}));
    // Ordering-only native coupon: it observes the stable sort before this
    // module's explicit duplicate-key rejection; it grants no source authority.
    EXPECT_EQ(NativeOrder(input, {-4,-4}), (std::vector<std::uint32_t>{0,1}));
    classified.roles[1].state = RoleState::CoatingReversed;
    EXPECT_THROW(SurfaceOrder(input, classified), std::exception);
}
} // namespace crash::cases::vehicle_self_contact::native::coated::test
