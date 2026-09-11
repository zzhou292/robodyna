#include "TinyFixture.h"
#include <type_traits>

namespace crash::modelio::tied_shell::auxiliary_test {
TEST(TiedAuxiliaryValues, RetainsCompleteGroupsRawBlankFieldsAndBothRoles) {
    static_assert(std::is_nothrow_copy_constructible_v<TiedAuxiliaryConstraints>);
    static_assert(!std::is_copy_assignable_v<TiedAuxiliaryConstraints>);
    Tiny fixture;
    const auto d = fixture.Prepare();
    EXPECT_EQ(d.member_sha256, output::Sha256(fixture.member));
    ASSERT_EQ(d.groups.size(), 2u);
    EXPECT_FALSE(d.groups[0].source_fields[1]);
    ASSERT_TRUE(d.groups[1].source_fields[1]);
    EXPECT_EQ(*d.groups[1].source_fields[1], 0.);
    EXPECT_EQ(d.groups[0].evidence.source_nodes, (std::vector<SourceId>{10,901,30}));
    EXPECT_EQ(d.groups[0].evidence.master_nodes, (std::vector<SourceId>{10}));
    EXPECT_EQ(d.groups[0].evidence.slave_nodes, (std::vector<SourceId>{30}));
    EXPECT_EQ(d.counts.member_occurrences, 5u);
    EXPECT_EQ(d.counts.distinct_members, 5u);
    EXPECT_EQ(d.counts.groups_touching_masters, 2u);
    EXPECT_EQ(d.counts.groups_touching_slaves, 1u);
    EXPECT_EQ(d.counts.distinct_master_members, 2u);
    EXPECT_EQ(d.counts.distinct_slave_members, 1u);
    EXPECT_EQ(d.counts.auxiliary_member_nodes, 2u);
    ASSERT_EQ(d.member_nodes.size(), 5u);
    EXPECT_EQ(d.member_nodes[0].canonical_index, 1u);
    EXPECT_EQ(d.member_nodes[3].id, 901u);
    EXPECT_EQ(d.member_nodes[3].origin, AuxiliaryNodeOrigin::AuxiliaryMember);
    EXPECT_EQ(d.member_nodes[3].auxiliary_code_blank_mask, 3u);
    EXPECT_EQ(d.member_nodes[4].auxiliary_code_blank_mask, 0u);
    for (const auto& source : d.sources)
        EXPECT_EQ(output::Sha256(source.block.raw_text), source.block.sha256);
    EXPECT_GT(d.startup_budget_bytes, d.owned_payload_bytes);
}
TEST(TiedAuxiliaryValues, WallReplacementIsExplicitCompleteAndOtherConstraintsStayUnresolved) {
    Tiny fixture;
    const auto retained = fixture.Prepare(OriginalWallPolicy::RetainUnresolved);
    const auto replaced = fixture.Prepare(OriginalWallPolicy::ReplaceWithMeshWall);
    EXPECT_EQ(retained.wall_policy, OriginalWallPolicy::RetainUnresolved);
    EXPECT_EQ(replaced.wall_policy, OriginalWallPolicy::ReplaceWithMeshWall);
    ASSERT_EQ(replaced.original_walls.size(), 1u);
    EXPECT_EQ(retained.original_walls[0].sha256, replaced.original_walls[0].sha256);
    ASSERT_EQ(replaced.wall_source_files.size(), 1u);
    EXPECT_EQ(replaced.wall_source_files[0].filename, "wall.key");
    ASSERT_EQ(replaced.other_unresolved_constraints.size(), 2u);
    bool joint = false, tie = false;
    for (const auto& block : replaced.other_unresolved_constraints) {
        joint |= block.keyword == "*CONSTRAINED_JOINT_SPHERICAL_ID";
        tie |= block.keyword == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE";
        EXPECT_NE(block.retained_source, SIZE_MAX);
    }
    EXPECT_TRUE(joint);
    EXPECT_TRUE(tie);
}
TEST(TiedAuxiliaryValues, RejectsLateAuthenticationAndMissingNodeWithExactCleanRetry) {
    Tiny fixture;
    const auto retained = fixture.Prepare();
    const auto hash = retained.sources.back().block.sha256;
    auto changed = fixture;
    changed.member.back() = 'x';
    EXPECT_THROW(changed.Prepare(), std::exception);
    changed = fixture;
    changed.AlterMember([](auto& raw) {
        const auto at = raw.find("             6.0");
        ASSERT_NE(at, std::string::npos);
        raw.replace(at, 16, "             7.0");
    });
    EXPECT_THROW(changed.Prepare(), std::exception); // Member rehashed, last block stale.
    changed = fixture;
    changed.AlterMember([](auto& raw) {
        const auto at = raw.find("       903");
        ASSERT_NE(at, std::string::npos);
        raw.replace(at, 10, "       999");
    }, true);
    EXPECT_THROW(changed.Prepare(), std::exception);
    EXPECT_EQ(retained.sources.back().block.sha256, hash);
    const auto retry = fixture.Prepare();
    EXPECT_EQ(retry.groups.back().evidence.source_nodes, retained.groups.back().evidence.source_nodes);
    EXPECT_EQ(retry.sources.back().cards, retained.sources.back().cards);
}
TEST(TiedAuxiliaryValues, RejectsSourceNamespaceAmbiguityAndNondefaultOptions) {
    Tiny fixture;
    for (unsigned fault = 0; fault < 3; ++fault) {
        auto changed = fixture;
        changed.AlterMember([&](auto& raw) {
            if (fault == 0) {
                const auto at = raw.find("*SET_NODE_LIST\n       912");
                ASSERT_NE(at, std::string::npos);
                raw.replace(at+15, 10, "       910");
            } else if (fault == 1) {
                const auto at = raw.find("       902         0");
                ASSERT_NE(at, std::string::npos);
                raw.replace(at+10, 10, "         7");
            } else {
                const auto at = raw.find("\n     903");
                ASSERT_NE(at, std::string::npos);
                raw.replace(at+1, 8, "     901");
            }
        }, true);
        EXPECT_THROW(changed.Prepare(), std::exception) << fault;
    }
}
TEST(TiedAuxiliaryValues, CapsAndWallPolicyFailuresPrecedePublication) {
    Tiny fixture;
    AuxiliaryLimits limits;
    limits.member_bytes = fixture.member.size()-1;
    EXPECT_THROW(fixture.Prepare(OriginalWallPolicy::ReplaceWithMeshWall, limits), std::exception);
    limits = {};
    limits.groups = 1;
    EXPECT_THROW(fixture.Prepare(OriginalWallPolicy::ReplaceWithMeshWall, limits), std::exception);
    limits = {};
    limits.group_members = 4;
    EXPECT_THROW(fixture.Prepare(OriginalWallPolicy::ReplaceWithMeshWall, limits), std::exception);
    EXPECT_THROW(fixture.Prepare(static_cast<OriginalWallPolicy>(77)), std::exception);
    auto changed = fixture;
    changed.AlterCanonical([](auto& doc) { doc["source_files"]["wall.key"]["blocks"].Clear(); });
    EXPECT_THROW(changed.Prepare(), std::exception);
    changed = fixture;
    changed.AlterCanonical([](auto& doc) {
        doc["source_files"][auxiliary_detail::MemberName]["keyword_counts"]["*CONSTRAINED_NODAL_RIGID_BODY"].SetUint(3);
    });
    EXPECT_THROW(changed.Prepare(), std::exception);
    EXPECT_EQ(fixture.Prepare().counts.groups, 2u);
}
} // namespace crash::modelio::tied_shell::auxiliary_test
