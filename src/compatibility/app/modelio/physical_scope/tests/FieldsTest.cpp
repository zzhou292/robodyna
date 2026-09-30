#include "../Internal.h"
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>

namespace crash::modelio::physical_scope {
namespace {
std::string Card(std::initializer_list<std::string> fields) {
    std::ostringstream out;
    for (const auto& value : fields) out << std::setw(10) << value;
    return out.str();
}
tied_shell::SourceEvidence Welds() {
    tied_shell::SourceEvidence source;
    source.block.keyword = "*CONSTRAINED_SPOTWELD_ID";
    source.cards = {{11, Card({"901"})}, {12, Card({"10", "30"})},
                    {14, Card({"902"})}, {15, Card({"30", "40", "1.25"})}};
    return source;
}
}
TEST(PhysicalScopeValues,LiteralWeldsRetainOrderAndOptionalCardsWithoutAdmission) {
    const auto input = Welds();
    const auto rows = detail::ReadSpotwelds({input}, {});
    ASSERT_EQ(rows.size(), 2);
    EXPECT_EQ(rows[0].id, 901);
    EXPECT_EQ(rows[1].nodes[1], 40);
    EXPECT_TRUE(rows[0].default_only);
    EXPECT_FALSE(rows[1].default_only);
    EXPECT_EQ(rows[1].first_card, 2);
    EXPECT_EQ(input.cards[3].second, Card({"30", "40", "1.25"}));
}
TEST(PhysicalScopeValues,LateWeldDefectsAndCapsPreservePreviousResult) {
    auto rows = detail::ReadSpotwelds({Welds()}, {});
    const auto original = rows.back().id;
    auto bad = Welds();
    bad.cards.back().second = Card({"30", "30"});
    EXPECT_THROW(rows = detail::ReadSpotwelds({bad}, {}), std::runtime_error);
    EXPECT_EQ(rows.back().id, original);
    bad = Welds(); bad.cards[2].second = Card({"901"});
    EXPECT_THROW(detail::ReadSpotwelds({bad}, {}), std::runtime_error);
    bad = Welds(); bad.cards.pop_back();
    EXPECT_THROW(detail::ReadSpotwelds({bad}, {}), std::runtime_error);
    Limits limits; limits.spotwelds = 1;
    EXPECT_THROW(detail::ReadSpotwelds({Welds()}, limits), std::runtime_error);
    limits.spotwelds = 2;
    EXPECT_NO_THROW(rows = detail::ReadSpotwelds({Welds()}, limits));
}
TEST(PhysicalScopeValues,SupplementalEndpointAndTireRolesDoNotBecomeBaselineMass) {
    Group group;
    group.members = {{30, Shell}, {10, ProvisionalType25 | ExcludedTireShell}, {20, BeamOrientation}};
    detail::Classify(group);
    EXPECT_EQ(group.before, Coverage::Partial);
    EXPECT_EQ(group.covered_before, 1);
    EXPECT_EQ(group.covered_after, 2);
    EXPECT_EQ(group.tire_members, 1);
    EXPECT_EQ(group.members[0].node, 30);
    group.members[2].roles |= RetainedPointMass;
    detail::Classify(group);
    EXPECT_EQ(group.after, Coverage::Complete);
    EXPECT_EQ(group.before, Coverage::Partial);
    group.members.back().node = 30;
    EXPECT_THROW(detail::Classify(group), std::runtime_error);
}
TEST(PhysicalScopeValues,UnknownNodeAndOverflowRejectBeforeMutation) {
    const std::vector<SourceId> nodes{10, 20, 40};
    std::vector<std::uint16_t> roles(nodes.size());
    EXPECT_THROW(detail::Mark(roles, nodes, 30, Type13Endpoint), std::runtime_error);
    EXPECT_EQ(roles, (std::vector<std::uint16_t>{0, 0, 0}));
    detail::Mark(roles, nodes, 40, Solid);
    EXPECT_EQ(roles[2], Solid);
    std::size_t bytes = 7;
    EXPECT_THROW(detail::Add(bytes, SIZE_MAX, 2, 100), std::runtime_error);
    EXPECT_EQ(bytes, 7);
    detail::Add(bytes, 3, 2, 13);
    EXPECT_EQ(bytes, 13);
}
} // namespace crash::modelio::physical_scope
