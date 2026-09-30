#include "../Internal.h"
#include "../../packing/Internal.h"
#include "../../tests/TinyFixture.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <type_traits>

namespace crash::modelio::tied_shell::test {
namespace {
using search_detail::RankedCoefficient;
struct Fixture : TinyFixture {
    Data declaration;
    PackingData packing;
    Fixture() : TinyFixture(false, true), declaration(Prepare()),
        packing(packing_detail::Build(canonical, declaration, {})) {}
    SearchGeometryData Geometry(SearchGeometryLimits limits = {}) const {
        return search_detail::Build(canonical, declaration, packing, member, limits);
    }
};
template<class T> void Replace(source::CanonicalData& source, const char* name, const std::vector<T>& values) {
    for (auto& a : source.arrays) {
        if (a.name != name) continue;
        a.descriptor.layout.rows = values.size()/a.descriptor.layout.columns;
        a.bytes = output::arrays::Encode(a.descriptor.layout, values.data(), values.size());
        a.descriptor.bytes = a.bytes.size();
        a.descriptor.sha256 = output::Sha256(a.bytes);
        return;
    }
    FAIL() << name;
}
}
TEST(TiedSearchGeometryValues, NativeRankUsesGeometryBeforeOverrideAndEquivalentFamilyTies) {
    for (const auto family : {SearchShellFamily::Q4, SearchShellFamily::T3}) {
        const std::vector<RankedCoefficient> values{{.5,250,99,0,family},
            {2.28,70000,0,0,family}, {2.28,70000,-0.,0,family}};
        std::vector<bool> winners;
        EXPECT_EQ(search_detail::ResolveEquivalent(values, winners), 2.28);
        EXPECT_EQ(winners, (std::vector<bool>{false,true,true}));
    }
    EXPECT_EQ(search_detail::ConsumedThickness({2,3,4,5}), 4);
    EXPECT_EQ(search_detail::ConsumedThickness({2,3,-0.,5}), 5);
    EXPECT_EQ(search_detail::ConsumedThickness({2,3,0,-0.}), 2);
}
TEST(TiedSearchGeometryValues, UnequalConsumedTiesAndMixedFamiliesPreserveOutput) {
    std::vector<bool> output{true,false,true,true};
    const auto prior = output;
    EXPECT_THROW(search_detail::ResolveEquivalent({{2,7,0,0},{2,7,0,3}}, output), std::exception);
    EXPECT_EQ(output, prior);
    EXPECT_THROW(search_detail::ResolveEquivalent({{2,7,0,0,SearchShellFamily::Q4},
        {2,7,0,0,SearchShellFamily::T3}}, output), std::exception);
    EXPECT_EQ(output, prior);
    EXPECT_THROW(search_detail::ConsumedThickness({2,7,-1,0}), std::exception);
    EXPECT_THROW(search_detail::ConsumedThickness({NAN,7,0,0}), std::exception);
}
TEST(TiedSearchGeometryValues, OriginalWorkingCoordinatesKeepSignedZeroAndLostRoundtripBits) {
    Fixture f;
    const auto d = f.Geometry();
    ASSERT_EQ(d.masters.size(), 2u);
    EXPECT_EQ(d.secondary_working_nodes.size(), 7u);
    EXPECT_EQ(d.working_positions.size(), 12u);
    EXPECT_TRUE(std::signbit(d.working_positions[0][0]));
    EXPECT_EQ(d.working_positions[1][0], 15.7);
    EXPECT_NE(d.working_positions[1][0], (15.7*.001)/.001);
    EXPECT_EQ(d.source_roundtrip_changed_components, 1u);
    EXPECT_EQ(d.maximum_secondary_shell_thickness, 0);
    EXPECT_EQ(d.equivalent_winner_occurrences, 2u);
    for (const auto& m : d.masters) {
        EXPECT_EQ(m.bounds_thickness, 1);
        EXPECT_EQ(m.projection_thickness, 1);
        EXPECT_EQ(m.declaration_row, f.packing.master_rows[&m-d.masters.data()]);
    }
    EXPECT_GT(d.startup_budget_bytes, d.owned_payload_bytes);
}
TEST(TiedSearchGeometryValues, LastOriginalCoordinateCodeAndLineMutationsRejectAndRetry) {
    Fixture clean;
    const auto accepted = clean.Geometry();
    for (unsigned fault = 0; fault < 4; ++fault) {
        Fixture f;
        if (fault == 0) {
            auto p = search_detail::Decode<double>(f.canonical, "node_positions");
            p.back() = -0.;
            Replace(f.canonical, "node_positions", p);
        } else if (fault == 1) {
            auto p = search_detail::Decode<std::uint32_t>(f.canonical, "node_source_lines");
            ++p.back();
            Replace(f.canonical, "node_source_lines", p);
        } else if (fault == 2) {
            auto p = search_detail::Decode<std::int32_t>(f.canonical, "node_codes");
            p.back() = 1;
            Replace(f.canonical, "node_codes", p);
        } else {
            f.canonical.inputs.units.length_to_m = .01;
        }
        EXPECT_THROW(f.Geometry(), std::exception) << fault;
    }
    const auto retry = clean.Geometry();
    EXPECT_EQ(retry.canonical_nodes, accepted.canonical_nodes);
    EXPECT_EQ(retry.working_positions, accepted.working_positions);
    EXPECT_EQ(retry.matches.size(), accepted.matches.size());
}
TEST(TiedSearchGeometryValues, CompleteOriginalIncidenceAndTriangleSubsetAreInspected) {
    Fixture f;
    auto topology = search_detail::ReadTopology(f.canonical, {});
    // A shell outside the selected master list still disqualifies a slave.
    topology.shells.insert(topology.shells.end(), {9999,999,30,60,40,50});
    topology.connectivity.insert(topology.connectivity.end(), {5,6,7,8});
    SearchGeometryData rejected;
    EXPECT_THROW(search_detail::Associate(topology, f.declaration, f.packing, rejected, {}), std::exception);
    topology = search_detail::ReadTopology(f.canonical, {});
    // The original triangle's three nodes are contained in this separate Q4.
    topology.shells.insert(topology.shells.end(), {9999,100,80,20,70,90});
    topology.connectivity.insert(topology.connectivity.end(), {2,3,4,0});
    SearchGeometryData next;
    search_detail::Associate(topology, f.declaration, f.packing, next, {});
    const auto triangle = std::find_if(next.masters.begin(), next.masters.end(), [](const auto& m) {
        return m.family == SearchShellFamily::T3;
    });
    ASSERT_NE(triangle, next.masters.end());
    EXPECT_EQ(triangle->match_count, 2u);
    EXPECT_EQ(next.matches[triangle->first_match+1].canonical_shell_row, 2u);
}
TEST(TiedSearchGeometryValues, ExactBudgetAndLateCardFailuresLeavePriorPreparationUsable) {
    static_assert(!std::is_copy_assignable_v<TiedShellSearchGeometry>);
    Fixture f;
    const auto accepted = f.Geometry();
    auto limits = SearchGeometryLimits{};
    limits.host_bytes = accepted.startup_budget_bytes-1;
    EXPECT_THROW(f.Geometry(limits), std::exception);
    limits.host_bytes = accepted.startup_budget_bytes;
    EXPECT_EQ(f.Geometry(limits).startup_budget_bytes, accepted.startup_budget_bytes);
    limits = {};
    limits.masters = 1;
    EXPECT_THROW(f.Geometry(limits), std::exception);
    auto altered = f.member;
    altered.back() = '!';
    EXPECT_THROW(search_detail::Build(f.canonical, f.declaration, f.packing, altered, {}), std::exception);
    Fixture bad_card;
    bad_card.AlterScope([](output::Document& doc) {
        auto& text = doc["declarations"]["tables"]["section"][1]["cards"][1]["text"];
        text.SetString("  2.000000", doc.GetAllocator());
    });
    EXPECT_THROW(bad_card.Geometry(), std::exception);
    EXPECT_EQ(f.Geometry().secondary_working_nodes, accepted.secondary_working_nodes);
}
} // namespace crash::modelio::tied_shell::test
