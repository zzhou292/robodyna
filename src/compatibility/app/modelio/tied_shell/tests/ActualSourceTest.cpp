#include "../TiedShellDeclaration.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
#include <gtest/gtest.h>
#include <iostream>
#include <numeric>
#include <type_traits>

namespace crash::modelio::tied_shell::test {
namespace {
const source::CanonicalSource& Canonical() {
    static const auto value = [] {
        auto input = source::test::ActualInputs();
        input.scope_report.bytes = 13212691;
        input.scope_report.sha256 = "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0";
        return source::CanonicalSource::Read(input);
    }();
    return value;
}
const std::string& Member() {
    static const auto bytes = output::ReadBounded(std::getenv("ROBO_STATIC_MEMBER"), 42846753);
    return bytes;
}
const TiedShellDeclaration& Declaration() {
    static const auto value = [] {
        const auto budget = TiedShellDeclaration::Forecast(Canonical());
        std::cout << "Tied source startup forecast: " << budget << " bytes\n";
        return TiedShellDeclaration::Prepare(Canonical(), Member());
    }();
    return value;
}
template<class T> std::vector<T> Decode(const char* name) {
    const auto& array = source::FindArray(Canonical().data(), name);
    return output::arrays::Decode<T>(array.descriptor, array.bytes);
}
}
TEST(TiedDeclarationActual, CompleteOriginalCensusRetainsAllTopologyAndConstraintEvidence) {
    const auto& prepared = Declaration();
    const auto& d = prepared.data();
    EXPECT_NE(&prepared.canonical(), &Canonical());
    EXPECT_EQ(&prepared.canonical().data(), &Canonical().data());
    EXPECT_EQ(d.slave_set_id, 2000002u);
    EXPECT_EQ(d.master_set_id, 2000001u);
    EXPECT_EQ(d.parts.size(), 361u);
    EXPECT_EQ(d.master_part_ids.size(), 359u);
    EXPECT_EQ(d.slave_part_ids, (std::vector<SourceId>{2000486,2000977}));
    EXPECT_EQ(d.counts.masters, 171813u);
    EXPECT_EQ(d.counts.q4, 160896u);
    EXPECT_EQ(d.counts.t3, 10917u);
    EXPECT_EQ(d.counts.master_nodes, 183457u);
    EXPECT_EQ(d.counts.slave_shells, 0u);
    EXPECT_EQ(d.counts.slave_beams, 4442u);
    EXPECT_EQ(d.counts.slave_solids, 908u);
    EXPECT_EQ(d.counts.slave_nodes, 11165u);
    EXPECT_EQ(d.counts.incidences, 16148u);
    EXPECT_EQ(d.counts.shared_nodes, 0u);
    EXPECT_EQ(d.counts.groups, 759u);
    EXPECT_EQ(d.counts.groups_touching_slaves, 0u);
    EXPECT_EQ(d.counts.groups_touching_masters, 409u);
    ASSERT_FALSE(d.masters.empty());
    EXPECT_EQ(d.masters.front().id, 2101172u);
    EXPECT_EQ(d.masters.back().id, 2492898u);
    EXPECT_EQ(d.slaves.front().id, 2102273u);
    EXPECT_EQ(d.slaves[4441].id, 2409378u);
    EXPECT_EQ(d.slaves[4442].id, 2200784u);
    EXPECT_EQ(d.slaves.back().id, 2201691u);
    std::size_t auxiliary_groups = 0, master_group_members = 0;
    for (const auto& group : d.groups) master_group_members += group.master_nodes.size();
    EXPECT_EQ(master_group_members, 2309u);
    for (const auto& block : d.unresolved_constraints) {
        if (block.filename == "yaris-coarse-v1l.key") {
            ASSERT_LT(block.retained_source, d.sources.size());
            EXPECT_EQ(d.sources[block.retained_source].block.sha256, block.sha256);
        } else {
            EXPECT_EQ(block.retained_source, SIZE_MAX);
            if (block.keyword == "*CONSTRAINED_NODAL_RIGID_BODY") ++auxiliary_groups;
        }
    }
    EXPECT_EQ(auxiliary_groups, 11u);
    EXPECT_EQ(d.ordering, NativeReadiness::Unresolved);
    EXPECT_EQ(d.classification, NativeReadiness::Unresolved);
    EXPECT_EQ(d.search, NativeReadiness::Unresolved);
    const auto records = Decode<SourceId>("shells_records");
    for (const auto& parent : d.masters) {
        const auto* row = records.data()+6*parent.canonical_index;
        EXPECT_EQ(row[0], parent.id);
        EXPECT_EQ(row[1], d.parts[parent.part_index].id);
        EXPECT_EQ(parent.arity, row[4] == row[5] ? 3u : 4u);
    }
    const auto ids = Decode<SourceId>("node_ids");
    SourceId previous = 0;
    for (const auto& node : d.slave_nodes) {
        EXPECT_GT(node.id, previous);
        EXPECT_EQ(node.id, ids.at(node.canonical_index));
        EXPECT_EQ(node.source_codes, (std::array<std::int32_t,2>{0,0}));
        previous = node.id;
    }
    const auto beams = Decode<SourceId>("beams_records");
    const auto solids = Decode<SourceId>("solids_records");
    for (const auto& incidence : d.incidences) {
        const auto& element = d.slaves.at(incidence.element);
        const auto& family = element.family == ElementFamily::Beam ? beams : solids;
        const auto* row = family.data()+10*element.canonical_index;
        EXPECT_EQ(row[0], element.id);
        EXPECT_EQ(row[2+incidence.local_slot], ids.at(incidence.canonical_node));
        EXPECT_LT(incidence.local_slot, element.arity);
    }
    std::cout << "Tied retained payload: " << d.owned_payload_bytes << " bytes\n";
}
TEST(TiedDeclarationActual, ImmutableBackingBudgetFailureAndLateCapRetry) {
    static_assert(!std::is_copy_assignable_v<TiedShellDeclaration>);
    const auto& prior = Declaration();
    const auto* data = &prior.data();
    auto limits = Limits{};
    limits.host_bytes = TiedShellDeclaration::Forecast(Canonical())-1;
    try {
        TiedShellDeclaration::Prepare(Canonical(), {}, limits);
        FAIL();
    } catch (const std::exception& error) {
        EXPECT_NE(std::string(error.what()).find("host byte cap"), std::string::npos);
    }
    limits = {};
    limits.masters = 171812;
    EXPECT_THROW(TiedShellDeclaration::Prepare(Canonical(), Member(), limits), std::exception);
    EXPECT_EQ(&prior.data(), data);
    EXPECT_EQ(prior.data().slaves.back().id, 2201691u);
    auto retry = [&] {
        auto local = Canonical();
        return TiedShellDeclaration::Prepare(local, Member());
    }();
    EXPECT_EQ(&retry.canonical().data(), &Canonical().data());
    EXPECT_EQ(retry.data().counts.incidences, prior.data().counts.incidences);
    ASSERT_EQ(retry.data().sources.size(), prior.data().sources.size());
    for (std::size_t i = 0; i < retry.data().sources.size(); ++i) {
        EXPECT_EQ(retry.data().sources[i].block.raw_text, prior.data().sources[i].block.raw_text);
        EXPECT_EQ(retry.data().sources[i].cards, prior.data().sources[i].cards);
    }
}
} // namespace crash::modelio::tied_shell::test
