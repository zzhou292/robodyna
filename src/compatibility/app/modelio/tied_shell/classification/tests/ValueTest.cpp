#include "Fixture.h"
#include "../../auxiliary/Internal.h"
#include <type_traits>

namespace crash::modelio::tied_shell::classification_test {
TEST(TiedClassificationSourceValues, AuxiliaryReaderRetainsCompleteSolidTopologyCards) {
    Fixture fixture;
    output::Document metadata;
    metadata.Parse(fixture.canonical.canonical_bytes.c_str());
    std::string member;
    for (const auto& source : fixture.auxiliary.sources) member += source.block.raw_text;
    const auto& file = metadata["source_files"][fixture.auxiliary.filename.c_str()];
    const auto sources = auxiliary_detail::Sources(file,member,{});
    ASSERT_EQ(sources.size(),fixture.auxiliary.sources.size());
    EXPECT_EQ(sources.back().block.keyword,"*ELEMENT_SOLID");
    EXPECT_EQ(sources.back().cards,fixture.auxiliary.sources.back().cards);
    EXPECT_EQ(sources.back().block.raw_text,fixture.auxiliary.sources.back().block.raw_text);
}
TEST(TiedClassificationSourceValues, CompleteRolesRetainOriginalNodesAndExplicitWholeWallReplacement) {
    Fixture fixture;
    const auto receipt = fixture.Check();
    EXPECT_EQ(receipt.observed_slaves,2u);
    EXPECT_EQ(receipt.plain_groups,1u);
    EXPECT_EQ(receipt.auxiliary_groups,1u);
    EXPECT_EQ(receipt.rigid_parts,1u);
    EXPECT_EQ(receipt.joints,1u);
    EXPECT_EQ(receipt.checked_joint_node_fields,5u);
    EXPECT_EQ(receipt.original_type2_interfaces,1u);
    EXPECT_EQ(receipt.replaced_primitive_walls,1u);
    EXPECT_EQ(receipt.wall.part_id,1001u);
    EXPECT_EQ(receipt.wall.node_ids,(std::vector<SourceId>{101,102,103,104}));
    EXPECT_EQ(receipt.wall.shell_ids,(std::vector<SourceId>{1001}));
    EXPECT_EQ(receipt.wall.sha256,output::Sha256(fixture.wall));
    for (const auto& source : receipt.wall.sources)
        EXPECT_EQ(output::Sha256(source.block.raw_text),source.block.sha256);
    static_assert(!std::is_copy_assignable_v<TiedClassificationContext>);
}
TEST(TiedClassificationSourceValues, EveryLateSlaveInfluenceRejectsIncludingPNODEAndFifthJointNode) {
    for (unsigned fault = 0; fault < 8; ++fault) {
        Fixture changed;
        if (fault == 0) changed.declaration.groups.back().source_nodes.push_back(30);
        if (fault == 1) changed.auxiliary.groups.back().evidence.source_nodes.push_back(30);
        if (fault == 2) changed.rigid.bodies.back().extra_nodes.push_back(30);
        if (fault == 3) changed.declaration.slave_nodes.back().source_codes[1] = 1;
        if (fault == 4) changed.declaration.sources[0].cards[0].second.replace(30,10,Card({30}));
        if (fault == 5) changed.rigid.sources[1].cards[1].second.replace(40,10,Card({30}));
        if (fault == 6) changed.rigid.sources[0].cards[0].second.replace(40,10,Card({1}));
        if (fault == 7) changed.auxiliary.sources.back().cards.back().second += Card({31},8);
        EXPECT_THROW(changed.Check(),std::exception) << fault;
    }
    EXPECT_NO_THROW(Fixture{}.Check());
}
TEST(TiedClassificationSourceValues, UnknownConsumedRolesMissingCardsAndAlteredCensusFailClosed) {
    for (const auto* keyword : {"*BOUNDARY_SPC_NODE","*CONSTRAINED_INTERPOLATION",
                               "*DATABASE_CROSS_SECTION_PLANE","*BOUNDARY_CYCLIC","*ELEMENT_SOLID_TET10"}) {
        Fixture changed;
        changed.declaration.sources.back().block.keyword = keyword;
        changed.Metadata();
        EXPECT_THROW(changed.Check(),std::exception) << keyword;
    }
    Fixture changed;
    changed.rigid.sources.erase(changed.rigid.sources.begin()+1);
    EXPECT_THROW(changed.Check(),std::exception);
    changed = Fixture{};
    auto& bytes = changed.canonical.canonical_bytes;
    const auto at = bytes.find("*CONSTRAINED_JOINT_SPHERICAL_ID\": 1");
    ASSERT_NE(at,std::string::npos);
    bytes[at + std::string("*CONSTRAINED_JOINT_SPHERICAL_ID\": ").size()] = '2';
    EXPECT_THROW(changed.Check(),std::exception);
}
TEST(TiedClassificationSourceValues, WallLateHashTopologyAndPolicyFailuresPreserveEarlierReceipt) {
    Fixture fixture;
    const auto saved = fixture.Check();
    auto changed = fixture;
    changed.wall.back() = 'x';
    EXPECT_THROW(changed.Check(),std::exception);
    changed = fixture;
    changed.wall_sources[4].cards[0].second.replace(40,8,Card({999},8));
    auto& shell = changed.wall_sources[4];
    shell.block.raw_text = shell.block.keyword + "\n" + shell.cards[0].second + "\n";
    shell.block.sha256 = output::Sha256(shell.block.raw_text);
    changed.Metadata();
    EXPECT_THROW(changed.Check(),std::exception);
    changed = fixture;
    changed.auxiliary.wall_policy = OriginalWallPolicy::RetainUnresolved;
    EXPECT_THROW(changed.Check(),std::exception);
    ClassificationSourceLimits cap;
    cap.wall_member_bytes = fixture.wall.size()-1;
    EXPECT_THROW(classification_detail::Wall(fixture.canonical,fixture.declaration,fixture.wall,cap),std::exception);
    ++cap.wall_member_bytes;
    EXPECT_EQ(classification_detail::Wall(fixture.canonical,fixture.declaration,fixture.wall,cap).sha256,saved.wall.sha256);
    EXPECT_EQ(fixture.Check().wall.node_ids,saved.wall.node_ids);
}
}
