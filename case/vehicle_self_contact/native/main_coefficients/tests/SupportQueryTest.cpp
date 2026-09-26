#include "Fixture.h"
#include "../SupportQuery.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
namespace {
std::size_t NativeQuery(const Fixture& fixture, const coated::s::Main& query, const std::vector<unsigned>& order) {
    std::array<unsigned, 4> face;
    std::copy_n(query.nodes, 4, face.begin());
    const auto result = NativeSupport(fixture.Native(order), face);
    return result.selected == SIZE_MAX ? SIZE_MAX : order[result.selected];
}
}
TEST(MixedMainSupportQuery, TriangleSubsetUsesWholeNativeQuadSearchAndOldTieCertificate) {
    Fixture fixture;
    auto query = fixture.Main();
    query.nodes[3] = query.nodes[2];
    const auto index = detail::PrepareSupportQueries(fixture.input, fixture.packed);
    const auto result = detail::QuerySupport(fixture.input, fixture.packed, index, query, true);
    ASSERT_EQ(result.owner, 0u);
    EXPECT_EQ(result.proof, OwnerProof::NativeMaterialGroupOrder);
    EXPECT_EQ(result.winners.size(), 3u);
    EXPECT_EQ(result.owner, NativeQuery(fixture, query, StorageOrder(fixture)));
    // Native Q4 first-equal selection is real, not a triangle last-equal rule.
    EXPECT_EQ(NativeQuery(fixture, query, {2,1,0}), 2u);
    EXPECT_EQ(detail::QuerySupport(fixture.input, fixture.packed, index, query, false).owner, SIZE_MAX);
}
TEST(MixedMainSupportQuery, ExistingT3TakesPriorityOverAThickerQuad) {
    Fixture fixture;
    auto& triangle = fixture.input.shells[1].primary;
    triangle.layout = n::ShellLayout::Triangle3;
    triangle.nodes[3] = triangle.nodes[2];
    fixture.packed.parts[0].coefficient.property_thickness = 100.;
    fixture.packed.parts[2].coefficient.property_thickness = 200.;
    fixture.Refresh();
    auto query = fixture.Main();
    query.nodes[3] = query.nodes[2];
    const auto index = detail::PrepareSupportQueries(fixture.input, fixture.packed);
    const auto selected = detail::QuerySupport(fixture.input, fixture.packed, index, query, true);
    ASSERT_EQ(selected.owner, 1u);
    ASSERT_EQ(selected.winners.size(), 1u);
    const auto native = NativeSupport(fixture.Native({0,1,2}), {0,1,2,2});
    EXPECT_EQ(native.q4, 2u);
    EXPECT_EQ(native.t3, 1u);
    EXPECT_EQ(native.selected, selected.owner);
    for (const auto order : {std::vector<unsigned>{0,1,2}, std::vector<unsigned>{2,1,0}})
        EXPECT_EQ(NativeQuery(fixture, query, order), selected.owner);
}
TEST(MixedMainSupportQuery, MissingPhysicalShellRemainsExplicitWithoutFabricatedOwner) {
    Fixture fixture;
    fixture.input.nodes.push_back({5,4,{2,1,0}});
    const unsigned shifted[]{1,2,3,4};
    std::copy_n(shifted, 4, fixture.input.shells[1].primary.nodes);
    fixture.Refresh();
    auto query = fixture.Main();
    query.nodes[0] = 0;
    query.nodes[1] = 2;
    query.nodes[2] = 4;
    query.nodes[3] = 4;
    const auto index = detail::PrepareSupportQueries(fixture.input, fixture.packed);
    const auto selected = detail::QuerySupport(fixture.input, fixture.packed, index, query, true);
    EXPECT_TRUE(selected.winners.empty());
    EXPECT_EQ(selected.owner, SIZE_MAX);
    EXPECT_EQ(NativeQuery(fixture, query, {2,1,0}), SIZE_MAX);
}
TEST(MixedMainSupportQuery, LegacyQueriesAndCornerPrecedenceAreUnchanged) {
    for (const bool triangle : {false, true}) {
        Fixture fixture(triangle);
        fixture.packed.parts[0].coefficient.young = 1.;
        auto& moved = fixture.input.shells[1].primary.nodes;
        if (triangle) { moved[0]=1; moved[1]=2; moved[2]=0; moved[3]=0; }
        else { moved[0]=1; moved[1]=2; moved[2]=3; moved[3]=0; }
        fixture.Refresh();
        const auto index = detail::PrepareSupportQueries(fixture.input, fixture.packed);
        for (const bool grouping : {false, true}) {
            const auto before = fixture.Select(grouping);
            const auto after = detail::QuerySupport(fixture.input, fixture.packed, index, fixture.Main(), grouping);
            EXPECT_EQ(before.owner, after.owner);
            EXPECT_EQ(before.proof, after.proof);
            EXPECT_EQ(before.winners, after.winners);
        }
        const auto selected = detail::QuerySupport(fixture.input, fixture.packed, index, fixture.Main(), false);
        ASSERT_NE(selected.owner, SIZE_MAX);
        EXPECT_EQ(selected.owner, NativeQuery(fixture, fixture.Main(), {1,2,0}));
    }
}
TEST(MixedMainSupportQuery, IndexCapAndForeignSourceRejectWithoutChangingInput) {
    Fixture fixture;
    const auto need = detail::SupportQueryBytes(fixture.input.nodes.size(), fixture.input.shells.size());
    const auto index = detail::PrepareSupportQueries(fixture.input, fixture.packed, need);
    EXPECT_THROW(detail::PrepareSupportQueries(fixture.input, fixture.packed, need-1), detail::Failure);
    EXPECT_EQ(fixture.input.shells[0].primary.source_id, 1000u);
    auto foreign = fixture.input;
    EXPECT_THROW(detail::QuerySupport(foreign, fixture.packed, index, fixture.Main(), true), std::exception);
    auto query = fixture.Main();
    query.nodes[0] = UINT32_MAX;
    EXPECT_THROW(detail::QuerySupport(fixture.input, fixture.packed, index, query, true), std::exception);
    EXPECT_THROW(detail::SupportQueryBytes(524289, 1), detail::Failure);
}
}
