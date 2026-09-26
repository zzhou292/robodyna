#include "../Internal.h"
#include "../InterfaceProfile.h"
#include "../SourceIds.h"
#include "modelio/native_spring_ids/tests/Fixture.h"
#include <gtest/gtest.h>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::test {
namespace d = detail;
namespace {
template<class Call> Report Rejected(Call call) {
    try { call(); } catch (const d::Failure& failure) { return failure.report; }
    ADD_FAILURE() << "Expected source rejection";
    return {};
}
}
TEST(EffectiveContactControl, SingleMidPropertySharingExpandsControlWithoutPidConstants) {
    const std::vector<d::Part> parts{{91, 401, 701, "one", 1}, {19, 401, 701, "two", 2}, {44, 802, 902, "one", 3}};
    const std::vector<d::Section> sections{{401, "*SECTION_SOLID"}, {802, "*SECTION_SOLID"}};
    const auto result = d::ResolveSharing(parts, sections, {91});
    ASSERT_EQ(result.size(), 3u);
    EXPECT_EQ(result[0].part_id, 19u);
    EXPECT_TRUE(result[0].effective_control);
    EXPECT_FALSE(result[0].directly_requested);
    EXPECT_EQ(result[0].native_property_id, 401u);
    EXPECT_FALSE(result[1].effective_control);
    EXPECT_TRUE(result[2].directly_requested);
    EXPECT_TRUE(result[2].effective_control);
}
TEST(EffectiveContactControl, ControlledMultiMidSharingRequiresNativePropertyMapping) {
    const std::vector<d::Part> parts{{91, 401, 701, "one", 1}, {19, 401, 702, "two", 2}};
    const std::vector<d::Section> sections{{401, "*SECTION_SOLID"}};
    const auto failed = Rejected([&] { (void)d::ResolveSharing(parts, sections, {91}); });
    EXPECT_EQ(failed.status, Status::NeedsNativePropertyMapping);
    EXPECT_EQ(failed.source_file, "one");
    const auto disabled = d::ResolveSharing(parts, sections, {});
    ASSERT_EQ(disabled.size(), 2u);
    for (const auto& part : disabled) {
        EXPECT_FALSE(part.effective_control);
        EXPECT_EQ(part.native_property_id, 0u); // Unconsumed clone identity stays unavailable.
    }
}
TEST(EffectiveContactControl, MissingDuplicateAndWrongPropertyAssociationsReject) {
    const std::vector<d::Part> parts{{91, 401, 701, "one", 1}};
    const std::vector<d::Section> sections{{401, "*SECTION_SOLID"}};
    EXPECT_EQ(Rejected([&] { (void)d::ResolveSharing(parts, {}, {91}); }).status, Status::InvalidInput);
    EXPECT_EQ(Rejected([&] { (void)d::ResolveSharing({parts[0], parts[0]}, sections, {91}); }).status, Status::InvalidInput);
    EXPECT_EQ(Rejected([&] { (void)d::ResolveSharing(parts, {sections[0], sections[0]}, {91}); }).status, Status::InvalidInput);
    EXPECT_EQ(Rejected([&] { (void)d::ResolveSharing(parts, sections, {22}); }).status, Status::InvalidInput);
    EXPECT_EQ(Rejected([&] { (void)d::ResolveSharing(parts, {{401, "*SECTION_SHELL"}}, {91}); }).status, Status::UnsupportedSource);
}
TEST(EffectiveContactControl, InterfaceClosureIncludesWallsConstraintsAndUnknownGenerators) {
    InterfaceCensus census;
    for (const auto* keyword : {"*CONTACT_AUTOMATIC_SINGLE_SURFACE", "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE",
            "*CONTACT_INTERIOR", "*RIGIDWALL_PLANAR", "*RIGIDWALL_PLANAR_FINITE_ID",
            "*RIGIDWALL_PLANAR_FINITE_FORCES_ID", "*CONSTRAINED_JOINT_REVOLUTE_ID",
            "*CONSTRAINED_EXTRA_NODES_SET", "*AIRBAG_SIMPLE_AIRBAG_MODEL_ID"})
        d::CheckInterfaceKeyword(keyword, census, "source.key", 7);
    EXPECT_EQ(census.type25_sources, 1u);
    EXPECT_EQ(census.type2_sources, 1u);
    EXPECT_EQ(census.interior_sources, 1u);
    EXPECT_EQ(census.rigid_wall_sources, 3u);
    EXPECT_EQ(census.checked_source_blocks, 9u);
    EXPECT_EQ(census.disposition, InterfaceDisposition::Unresolved); // Only the whole private factory seals absence.
    for (const auto* keyword : {"*CONTACT_NEW_KIND", "*RIGIDWALL_UNKNOWN", "*CONSTRAINED_UNKNOWN",
            "*INCLUDE_RADIOSS", "*PART_SENSOR", "*AIRBAG_UNKNOWN"})
        EXPECT_EQ(Rejected([&] { d::CheckInterfaceKeyword(keyword, census, "bad.key", 8); }).status,
            Status::UnsupportedSource);
}
TEST(EffectiveContactControl, CommonNamespaceOffsetsAreExplicitAndMixedTransformsReject) {
    using modelio::native_spring_ids::test::Card;
    modelio::tied_shell::SourceEvidence source;
    source.block.keyword = "*INCLUDE_TRANSFORM";
    source.cards = {{1, "child.key"}, {2, Card({7,7,7,7,7,7,7})}, {3, Card({7})}, {4, ""}, {5, Card({1})}};
    EXPECT_EQ(d::Offset(source), 7);
    EXPECT_EQ(d::Shift(11, d::Offset(source)), 18u);
    source.cards[2].second = Card({8});
    EXPECT_EQ(Rejected([&] { (void)d::Offset(source); }).status, Status::NeedsNativePropertyMapping);
    EXPECT_EQ(Rejected([&] { (void)d::Shift(11, -12); }).status, Status::UnsupportedSource);
    EXPECT_EQ(Rejected([&] { (void)d::Shift(INT_MAX, 1); }).status, Status::UnsupportedSource);
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::test
