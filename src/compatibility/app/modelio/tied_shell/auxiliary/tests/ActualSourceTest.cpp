#include "../Internal.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
#include <gtest/gtest.h>
#include <iostream>

namespace crash::modelio::tied_shell::auxiliary_test {
namespace {
const source::CanonicalSource& Canonical() {
    static const auto source = [] {
        auto input = source::test::ActualInputs();
        input.scope_report.bytes = 13212691;
        input.scope_report.sha256 = "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0";
        return source::CanonicalSource::Read(input);
    }();
    return source;
}
const TiedShellDeclaration& Declaration() {
    static const auto declaration = [] {
        const char* path = std::getenv("ROBO_STATIC_MEMBER");
        output::Require(path, "Missing original main source fixture");
        return TiedShellDeclaration::Prepare(Canonical(), output::ReadBounded(path, 42846753));
    }();
    return declaration;
}
const std::string& Member() {
    static const auto bytes = [] {
        const char* path = std::getenv("ROBO_TIED_AUX_MEMBER");
        output::Require(path, "Missing original auxiliary source fixture");
        return output::ReadBounded(path, 44991);
    }();
    return bytes;
}
const TiedAuxiliaryConstraints& Prepared() {
    static const auto value = TiedAuxiliaryConstraints::Prepare(
        Declaration(), Member(), OriginalWallPolicy::ReplaceWithMeshWall);
    return value;
}
}
TEST(TiedAuxiliaryActual, CompleteOriginalGroupsAndExternalNodesPreserveMasterSlaveEvidence) {
    const auto& value = Prepared();
    const auto& d = value.data();
    EXPECT_EQ(&value.declaration().canonical().data(), &Canonical().data());
    EXPECT_EQ(&value.declaration().data(), &Declaration().data());
    EXPECT_EQ(d.counts.groups, 11u);
    EXPECT_EQ(d.counts.member_occurrences, 228u);
    EXPECT_EQ(d.counts.groups_touching_masters, 9u);
    EXPECT_EQ(d.counts.groups_touching_slaves, 0u);
    EXPECT_EQ(d.counts.distinct_master_members, 125u);
    EXPECT_EQ(d.counts.distinct_slave_members, 0u);
    EXPECT_EQ(d.counts.auxiliary_member_nodes, 88u);
    EXPECT_EQ(Declaration().data().counts.groups_touching_masters, 409u);
    ASSERT_EQ(d.groups.size(), 11u);
    for (std::size_t i = 0; i < 10; ++i) EXPECT_FALSE(d.groups[i].source_fields[1]);
    ASSERT_TRUE(d.groups.back().source_fields[1]);
    EXPECT_EQ(*d.groups.back().source_fields[1], 0.);
    for (const auto& source : d.sources)
        EXPECT_EQ(output::Sha256(source.block.raw_text), source.block.sha256);
    for (const auto& node : d.member_nodes) {
        EXPECT_FALSE(node.slave);
        if (node.origin == AuxiliaryNodeOrigin::AuxiliaryMember) {
            EXPECT_EQ(node.canonical_index, UINT32_MAX);
            ASSERT_LT(node.source, d.sources.size());
            EXPECT_EQ(d.sources[node.source].block.keyword, "*NODE");
            EXPECT_GE(node.source_line, d.sources[node.source].block.first_line);
            EXPECT_LE(node.source_line, d.sources[node.source].block.last_line);
        }
    }
    EXPECT_EQ(d.original_walls.size(), 6u);
    EXPECT_EQ(d.wall_source_files.size(), 2u);
    std::cout << "Auxiliary startup forecast " << d.startup_budget_bytes
              << "; own payload " << d.owned_payload_bytes << " bytes\n";
}
TEST(TiedAuxiliaryActual, ImmutableCopyAndLateMemberFailurePreserveOriginalSourceScope) {
    auto copy = Prepared();
    auto moved = std::move(copy);
    EXPECT_EQ(&copy.data(), &moved.data());
    EXPECT_EQ(&moved.data(), &Prepared().data());
    auto changed = Member();
    changed.back() = changed.back() == 'x' ? 'y' : 'x';
    EXPECT_THROW(TiedAuxiliaryConstraints::Prepare(Declaration(), changed,
        OriginalWallPolicy::ReplaceWithMeshWall), std::exception);
    AuxiliaryLimits limits;
    limits.host_bytes = Prepared().data().startup_budget_bytes-1;
    EXPECT_THROW(TiedAuxiliaryConstraints::Prepare(Declaration(), Member(),
        OriginalWallPolicy::ReplaceWithMeshWall, limits), std::exception);
    const auto retry = TiedAuxiliaryConstraints::Prepare(Declaration(), Member(), OriginalWallPolicy::ReplaceWithMeshWall);
    EXPECT_EQ(retry.data().groups.back().evidence.source_nodes, moved.data().groups.back().evidence.source_nodes);
    EXPECT_EQ(retry.data().member_sha256, moved.data().member_sha256);
}
} // namespace crash::modelio::tied_shell::auxiliary_test
