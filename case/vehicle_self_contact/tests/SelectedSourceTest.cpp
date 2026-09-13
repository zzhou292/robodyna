#include "../SelectedSelfContactSource.h"
#include "lib_utest/qualification/self_contact_active_uses/Fixture.h"
#include <gtest/gtest.h>

namespace crash::cases::vehicle_self_contact::test {
namespace {

modelio::self_contact::Data Original() {
    using Part = modelio::self_contact::PartDisposition;
    modelio::self_contact::Data value;
    value.selected_part_ids = {2000524, 1001, 1000, 7000, 8000};
    value.parts = {
        Part{2000524, 2000524, 2000524, true, true, false, 2, 0, 0},
        Part{1001, 1001, 1001, true, true, false, 2, 0, 0},
        Part{1000, 1000, 1000, true, true, false, 2, 0, 0},
        Part{7000, 7000, 7000, true, false, true, 5, 0, 0},
        Part{8000, 8000, 8000, false, false, false, 0, 7, 2},
    };
    value.counts = {5, 4, 3, 1, 1, 11, 6, 5, 7, 2};
    return value;
}

struct OffsetFixture {
    qbat_catalog_test::Fixture source;
    tl::fea::ShellBatchBinding shells;
    tl::fea::ShellBatchPlasticityBinding catalog;
    tl::fea::ShellBatchFailureBinding failure;
    tl::fea::NodalNodeDomain domain;
    tl::fea::ShellNodeMap map;
    tl::fea::NodalCoefficientLedger ledger;
    tl::fea::ShellPhysicalBinding physical;
    OffsetFixture() {
        EXPECT_EQ(shells.InitializeFormulations(source.Geometry()).status,
            tl::fea::ShellBindingStatus::Success);
        EXPECT_EQ(catalog.InitializeFormulations(
            shells, source.Input()).status,
            tl::fea::ShellPlasticityBindingStatus::Success);
        EXPECT_EQ(failure.Initialize(catalog, source.failures.data(),
            source.failures.size()).status,
            tl::fea::ShellPlasticityBindingStatus::Success);
        std::vector<tl::fea::NodalDomainNode> nodes;
        for (std::size_t n = shells.node_count(); n-- > 0;) {
            const auto& node = shells.nodes()[n];
            nodes.push_back({node.source_id, node.position});
        }
        EXPECT_TRUE(domain.Initialize({77, nodes.data(), nodes.size()}));
        EXPECT_TRUE(map.Initialize(shells, domain));
        EXPECT_TRUE(ledger.Initialize({&map, nullptr, nullptr}));
        EXPECT_TRUE(physical.Initialize(
            {&shells, &catalog, &failure, nullptr}, ledger));
    }
};

void CheckLevel(unsigned level) {
    active_use_test::Fixture fixture(level);
    auto original = Original();
    const auto source = SelectedSelfContactSource::Prepare(
        fixture.physical, original, {}, {level});
    EXPECT_TRUE(source.prepared());
    EXPECT_TRUE(source.MatchesPhysical(fixture.physical));
    EXPECT_EQ(source.config().facet_level, level);
    EXPECT_EQ(source.inventory().selected_part_ids,
        (std::vector<std::uint64_t>{2000524, 1001, 1000}));
    EXPECT_EQ(source.inventory().retained_shell_part_ids,
        source.inventory().selected_part_ids);
    EXPECT_TRUE(source.inventory().offset_part_ids.empty());
    EXPECT_EQ(source.inventory().omitted_tire_part_ids,
        (std::vector<std::uint64_t>{7000}));
    EXPECT_EQ(source.inventory().unsupported_non_shell_part_ids,
        (std::vector<std::uint64_t>{8000}));
    EXPECT_EQ(source.inventory().unsupported_solid_part_ids,
        (std::vector<std::uint64_t>{8000}));
    EXPECT_EQ(source.inventory().unsupported_beam_part_ids,
        (std::vector<std::uint64_t>{8000}));
    const auto& counts = source.census().source;
    EXPECT_EQ(counts.original_selected_parts, 5u);
    EXPECT_EQ(counts.selected_shell_parts, 3u);
    EXPECT_EQ(counts.original_retained_shell_parts, 3u);
    EXPECT_EQ(counts.selected_offset_parts, 0u);
    EXPECT_EQ(counts.selected_shell_parents, 6u);
    EXPECT_EQ(counts.selected_offset_parents, 0u);
    EXPECT_EQ(counts.omitted_tire_parts, 1u);
    EXPECT_EQ(counts.omitted_tire_shells, 5u);
    EXPECT_EQ(counts.unsupported_solids, 7u);
    EXPECT_EQ(counts.unsupported_beams, 2u);
    EXPECT_EQ(counts.q4_parents, 3u);
    EXPECT_EQ(counts.t3_parents, 3u);

    contact::SelfContactActiveUseCounts expected;
    ASSERT_TRUE(contact::CountSelfContactActiveUses(
        {3, 3, source.surface().vertices().size(),
         source.surface().edges().size(), level}, &expected));
    const auto& topology = source.census().topology;
    EXPECT_EQ(topology.parents, expected.parents);
    EXPECT_EQ(topology.facets, expected.facets);
    EXPECT_EQ(topology.canonical_vertices, expected.vertices);
    EXPECT_EQ(topology.canonical_edges, expected.edges);
    EXPECT_EQ(topology.parent_local_vertex_uses, expected.vertex_uses);
    EXPECT_EQ(topology.parent_local_edge_uses, expected.edge_uses);
    EXPECT_EQ(source.facets().forecast().facets, expected.facets);
    EXPECT_EQ(source.active_uses().forecast().facets, expected.facets);
    EXPECT_EQ(source.census().support.vertex_uses.ordinary,
        expected.vertex_uses);
    EXPECT_EQ(source.census().support.edge_endpoints.ordinary,
        2 * expected.edge_uses);
    EXPECT_EQ(source.census().support.all_weighted_supports.total,
        expected.vertex_uses + 2 * expected.edge_uses);
    EXPECT_TRUE(source.census().support.complete_static_cin_roster);
    EXPECT_FALSE(source.census().support.runtime_activity_and_release_pending);
    EXPECT_EQ(source.census().reference_area.q4_parent_area_m2.lower > 0,
        true);
    EXPECT_EQ(source.census().reference_area.t3_parent_area_m2.lower > 0,
        true);

    original.selected_part_ids.clear();
    original.parts.clear();
    EXPECT_EQ(source.inventory().selected_part_ids.size(), 3u);
    EXPECT_EQ(source.surface().parents().size(), 6u);
    const auto copy = source;
    EXPECT_TRUE(copy.SharesStorage(source));
    active_use_test::Fixture independent(level);
    EXPECT_FALSE(source.MatchesPhysical(independent.physical));
}

}  // namespace

TEST(VehicleSelfContactValues,
     ImmutableSelectedSourceComposesAllThreeFixedLevels) {
    for (unsigned level = 0; level <= 2; ++level) CheckLevel(level);
}

TEST(VehicleSelfContactValues,
     NoncenteredSelectedLayersRemainExplicitAndNeverEnterFacets) {
    OffsetFixture fixture;
    const auto source = SelectedSelfContactSource::Prepare(
        fixture.physical, Original(), {}, {0});
    EXPECT_EQ(source.census().source.selected_shell_parents, 2u);
    EXPECT_EQ(source.census().source.selected_offset_parents, 4u);
    EXPECT_EQ(source.census().source.original_retained_shell_parts, 3u);
    EXPECT_EQ(source.census().source.selected_shell_parts, 1u);
    EXPECT_EQ(source.census().source.selected_offset_parts, 2u);
    EXPECT_EQ(source.inventory().selected_part_ids,
        (std::vector<std::uint64_t>{2000524}));
    EXPECT_EQ(source.inventory().offset_part_ids,
        (std::vector<std::uint64_t>{1001, 1000}));
    EXPECT_EQ(source.census().source.q4_parents, 1u);
    EXPECT_EQ(source.census().source.t3_parents, 1u);
    EXPECT_EQ(source.surface().parents().size(), 2u);
    EXPECT_EQ(source.inventory().offset_parents.size(), 4u);
    for (const auto& parent : source.surface().parents())
        EXPECT_TRUE(parent.source.source_parent_id == 103 ||
            parent.source.source_parent_id == 102);
}

TEST(VehicleSelfContactValues,
     DuplicatePidLateParentMismatchAndCapsRejectWithoutChangingSource) {
    active_use_test::Fixture fixture(2);
    auto original = Original();
    const auto saved = SelectedSelfContactSource::Prepare(
        fixture.physical, original, {}, {2});
    const auto identity = &saved.surface().parents()[0];

    auto duplicate = original;
    duplicate.selected_part_ids.back() =
        duplicate.selected_part_ids.front();
    duplicate.parts.back().part_id = duplicate.selected_part_ids.back();
    EXPECT_THROW(SelectedSelfContactSource::Preflight(
        fixture.physical, duplicate, {}, {2}), std::runtime_error);
    EXPECT_EQ(&saved.surface().parents()[0], identity);

    auto late = original;
    ++late.parts[2].shells;
    ++late.counts.shells;
    ++late.counts.retained_shells;
    EXPECT_THROW(SelectedSelfContactSource::Preflight(
        fixture.physical, late, {}, {2}), std::runtime_error);
    EXPECT_EQ(&saved.surface().parents()[0], identity);

    const auto forecast = SelectedSelfContactSource::Preflight(
        fixture.physical, original, {}, {2});
    Limits exact;
    exact.host_bytes = forecast.peak_host_bytes;
    EXPECT_EQ(SelectedSelfContactSource::Preflight(
        fixture.physical, original, {}, {2}, exact).peak_host_bytes,
        exact.host_bytes);
    --exact.host_bytes;
    EXPECT_THROW(SelectedSelfContactSource::Preflight(
        fixture.physical, original, {}, {2}, exact), std::runtime_error);
    EXPECT_EQ(&saved.surface().parents()[0], identity);
    ++exact.host_bytes;
    const auto retried = SelectedSelfContactSource::Prepare(
        fixture.physical, original, {}, {2}, exact);
    EXPECT_EQ(retried.forecast().peak_host_bytes, exact.host_bytes);

    Limits short_count;
    short_count.active_uses.max_facets =
        forecast.active_uses.facets - 1;
    EXPECT_THROW(SelectedSelfContactSource::Prepare(
        fixture.physical, original, {}, {2}, short_count),
        std::runtime_error);
    EXPECT_EQ(&saved.surface().parents()[0], identity);
}

}  // namespace crash::cases::vehicle_self_contact::test
