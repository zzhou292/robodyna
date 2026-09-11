#include "NativeOracle.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#ifdef ROBO_DYNA_NATIVE_BUCKET
#include "BucketOracle.h"
#include "lib_src/constraints/tied_shell/TiedSearch.h"
#include "lib_src/constraints/tied_shell/TiedPatchGeometry.h"
#include "lib_utest/qualification/tied_shell_search/NativeOracle.h"
#endif

namespace crash::modelio::tied_shell::test {
namespace {
const TiedShellSearchGeometry& ActualGeometry() {
    static const auto value = [] {
        auto inputs = source::test::ActualInputs();
        inputs.scope_report.bytes = 13212691;
        inputs.scope_report.sha256 = "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0";
        const auto canonical = source::CanonicalSource::Read(inputs);
        const auto member = output::ReadBounded(std::getenv("ROBO_STATIC_MEMBER"), 42846753);
        const auto declaration = TiedShellDeclaration::Prepare(canonical, member);
        const auto packing = TiedShellPacking::Prepare(declaration);
        std::cout << "Tied search geometry forecast before preparation: "
                  << TiedShellSearchGeometry::Forecast(packing) << " B\n" << std::flush;
        return TiedShellSearchGeometry::Prepare(packing, member);
    }();
    return value;
}
std::array<SourceId,4> Nodes(const std::vector<SourceId>& records, std::size_t row) {
    return {records.at(6*row+2),records.at(6*row+3),records.at(6*row+4),records.at(6*row+5)};
}
}
TEST(TiedSearchGeometryActual, CompleteOriginalWorkingGeometryPreservesDeclaredPatchesAndLayerSets) {
    const auto& prepared = ActualGeometry();
    const auto& d = prepared.data();
    const auto& packing = prepared.packing();
    const auto& declaration = packing.declaration().data();
    ASSERT_EQ(d.masters.size(), 171813u);
    EXPECT_EQ(d.canonical_nodes.size(), 194622u);
    EXPECT_EQ(d.working_positions.size(), 194622u);
    EXPECT_EQ(d.secondary_working_nodes.size(), 11165u);
    EXPECT_EQ(d.matches.size(), 180315u);
    EXPECT_EQ(d.multiple_match_masters, 4251u);
    EXPECT_EQ(d.equivalent_winner_occurrences, 176064u);
    EXPECT_EQ(d.maximum_secondary_shell_thickness, 0);
    EXPECT_EQ(declaration.search, NativeReadiness::Unresolved);
    EXPECT_EQ(declaration.classification, NativeReadiness::Unresolved);
    EXPECT_LE(d.startup_budget_bytes, SearchGeometryLimits{}.host_bytes);
    EXPECT_GT(d.startup_budget_bytes, d.owned_payload_bytes);
    const auto& canonical = packing.declaration().canonical().data();
    const auto records = search_detail::Decode<SourceId>(canonical, "shells_records");
    const auto indices = search_detail::Decode<std::uint32_t>(canonical, "shells_node_indices");
    std::size_t triples = 0, triple_q4 = 0, triple_t3 = 0;
    for (std::size_t rank = 0; rank < d.masters.size(); ++rank) {
        const auto& m = d.masters[rank];
        ASSERT_EQ(m.declaration_row, packing.data().master_rows[rank]);
        const auto& patch = declaration.masters.at(m.declaration_row);
        for (unsigned local = 0; local < 4; ++local)
            ASSERT_EQ(d.canonical_nodes.at(m.working_nodes[local]), indices.at(4*patch.canonical_index+local));
        if (m.match_count == 1) continue;
        ++triples;
        ++(m.family == SearchShellFamily::Q4 ? triple_q4 : triple_t3);
        ASSERT_EQ(m.match_count, 3u);
        ASSERT_EQ(records[6*patch.canonical_index+1], 2000524u);
        ASSERT_EQ(m.bounds_thickness, 2.28);
        ASSERT_EQ(m.projection_thickness, 2.28);
        std::vector<SourceId> winners;
        for (std::size_t i = 0; i < 3; ++i) {
            const auto& match = d.matches.at(m.first_match+i);
            const auto& p = d.properties.at(match.property);
            ASSERT_EQ(match.family, m.family);
            if (match.equivalent_winner) {
                winners.push_back(p.part_id);
                EXPECT_EQ(p.geometry_thickness, 2.28);
                EXPECT_EQ(p.rank_modulus, 70000);
                EXPECT_EQ(p.part_override, 0);
                EXPECT_EQ(p.element_override, 0);
            }
        }
        EXPECT_EQ(winners, (std::vector<SourceId>{2000023,2000523}));
    }
    EXPECT_EQ(triples, 4251u);
    EXPECT_EQ(triple_q4, 4250u);
    EXPECT_EQ(triple_t3, 1u);
    std::cout << "Tied search retained payload: " << d.owned_payload_bytes
              << " B; source round-trip changed components: " << d.source_roundtrip_changed_components << '\n';
}
TEST(TiedSearchGeometryActual, EveryNativeMatchAndAll4251TriplePermutationsHaveIdenticalConsumedInputs) {
    const auto& prepared = ActualGeometry();
    const auto& d = prepared.data();
    const auto& declaration = prepared.packing().declaration().data();
    const auto records = search_detail::Decode<SourceId>(prepared.packing().declaration().canonical().data(), "shells_records");
    std::size_t observed = 0, triple_permutations = 0;
    for (const auto& m : d.masters) {
        const auto master = Nodes(records, declaration.masters[m.declaration_row].canonical_index);
        std::vector<std::array<SourceId,4>> nodes;
        std::vector<search_detail::RankedCoefficient> coefficients;
        std::vector<unsigned> order(m.match_count);
        std::iota(order.begin(), order.end(), 0);
        for (unsigned i = 0; i < m.match_count; ++i) {
            const auto& match = d.matches[m.first_match+i];
            const auto& p = d.properties[match.property];
            nodes.push_back(Nodes(records, match.canonical_shell_row));
            coefficients.push_back({p.geometry_thickness,p.rank_modulus,p.part_override,p.element_override,match.family});
        }
        do {
            const auto native = NativeGeometry(master, nodes, coefficients, order);
            ASSERT_EQ(native.consumed[0], m.bounds_thickness) << m.declaration_row;
            ASSERT_EQ(native.consumed[1], m.projection_thickness) << m.declaration_row;
            const auto selected = native.selected[m.family == SearchShellFamily::Q4 ? 0 : 1];
            ASSERT_GE(selected, 0);
            ASSERT_TRUE(d.matches[m.first_match+selected].equivalent_winner);
            ++observed;
            if (m.match_count == 3) ++triple_permutations;
        } while (std::next_permutation(order.begin(), order.end()));
    }
    EXPECT_EQ(triple_permutations, 4251u*6);
    EXPECT_EQ(observed, 167562u+4251u*6);
}
TEST(TiedSearchGeometryActual, PublicLifetimeBudgetAndOriginalMemberRejectionPreservePriorAndRetry) {
    const auto& prior = ActualGeometry();
    const auto& packing = prior.packing();
    auto limits = SearchGeometryLimits{};
    limits.host_bytes = TiedShellSearchGeometry::Forecast(packing)-1;
    EXPECT_THROW(TiedShellSearchGeometry::Prepare(packing, "unconsumed", limits), std::exception);
    limits = {};
    limits.masters = 171812;
    EXPECT_THROW(TiedShellSearchGeometry::Prepare(packing, "unconsumed", limits), std::exception);
    auto member = output::ReadBounded(std::getenv("ROBO_STATIC_MEMBER"), 42846753);
    member.back() ^= 1;
    EXPECT_THROW(TiedShellSearchGeometry::Prepare(packing, member), std::exception);
    member.back() ^= 1;
    const auto retry = [&] {
        auto local = packing;
        return TiedShellSearchGeometry::Prepare(local, member);
    }();
    EXPECT_EQ(&retry.packing().declaration().canonical().data(), &packing.declaration().canonical().data());
    EXPECT_EQ(retry.data().canonical_nodes, prior.data().canonical_nodes);
    EXPECT_EQ(retry.data().working_positions, prior.data().working_positions);
    EXPECT_EQ(retry.data().matches.size(), prior.data().matches.size());
    EXPECT_EQ(retry.data().masters.back().projection_thickness, prior.data().masters.back().projection_thickness);
    EXPECT_FALSE(retry.PhysicalOwnNode(0, 0));
}
#ifdef ROBO_DYNA_NATIVE_BUCKET
TEST(TiedSearchGeometryActual, IndependentNativeBucketsRetainEveryOriginalSecondaryAndSelectedSourcePatch) {
    namespace ts = tl::constraints::tied_shell;
    const auto& geometry = ActualGeometry();
    const auto& d = geometry.data();
    const auto native = NativeBucket(geometry);
    ASSERT_EQ(native.selected.size(), 11165u);
    std::vector<int> last_master(native.selected.size(), 0);
    std::vector<std::size_t> counts(native.selected.size(), 0);
    int previous = 0;
    for (const auto& pair : native.pairs) {
        ASSERT_GE(pair[0], previous); previous = pair[0];
        ASSERT_GE(pair[0], 1); ASSERT_LE(std::size_t(pair[0]), d.masters.size());
        ASSERT_GE(pair[1], 1); ASSERT_LE(std::size_t(pair[1]), native.selected.size());
        ASSERT_NE(last_master[pair[1]-1], pair[0]);
        last_master[pair[1]-1] = pair[0];
        ++counts[pair[1]-1];
    }
    std::size_t matched = 0, outside = 0, singular = 0;
    double smallest = std::numeric_limits<double>::max(), largest = 0;
    for (std::size_t i = 0; i < native.selected.size(); ++i) {
        const auto selected = native.selected[i];
        if (!selected) continue;
        ++matched;
        ASSERT_GT(counts[i], 0u);
        ASSERT_GT(selected, 0); ASSERT_LE(std::size_t(selected), d.masters.size());
        const auto& m = d.masters[selected-1];
        ts::WorkingSearchInput input;
        input.working_length_to_m = d.working_length_to_m;
        input.master_thickness = m.projection_thickness;
        input.topology = m.family == SearchShellFamily::Q4 ?
            ts::MasterTopology::Quad : ts::MasterTopology::TriangleRepeatedThird;
        for (unsigned j = 0; j < 5; ++j) {
            const auto& p = d.working_positions[j == 4 ? d.secondary_working_nodes[i] : m.working_nodes[j]];
            const ts::Vec3 point{p[0],p[1],p[2]};
            if (j == 4) input.geometry.secondary_position = point;
            else input.geometry.master_position[j] = point;
        }
        ts::CandidateProjection projected;
        ASSERT_EQ(ts::ProjectCandidate(input, projected), ts::Status::Success);
        ASSERT_TRUE(projected.admissible);
        tied_search_test::NativeChoice independent;
        const auto native_projection = tied_search_test::Native(input, selected, independent);
        tied_search_test::Compare(projected, native_projection);
        ASSERT_FALSE(HasFailure()) << i;
        EXPECT_DOUBLE_EQ(independent.st[0], native.st[i][0]);
        EXPECT_DOUBLE_EQ(independent.st[1], native.st[i][1]);
        EXPECT_DOUBLE_EQ(independent.distance, native.distance[i]);
        outside += projected.outside_warning;
        smallest = std::min(smallest, projected.gap_m);
        largest = std::max(largest, projected.gap_m);
        ts::PatchInput patch_input = input.geometry;
        for (auto& p : patch_input.master_position) p = tl::math::fixed3::Scale(p, d.working_length_to_m);
        patch_input.secondary_position = tl::math::fixed3::Scale(patch_input.secondary_position, d.working_length_to_m);
        ts::Patch patch;
        singular += ts::PreparePatch(patch_input, patch) != ts::Status::Success;
    }
    EXPECT_GT(matched, 0u);
    std::cout << "Original native bucket pairs " << native.pairs.size()
              << "; cells " << native.cells[0] << ',' << native.cells[1] << ',' << native.cells[2]
              << "; matched " << matched << "; unmatched " << native.selected.size()-matched
              << "; outside " << outside << "; selected force patches rejected " << singular
              << "; gap_m " << smallest << ',' << largest << '\n';
    RecordProperty("native_pairs", std::to_string(native.pairs.size()));
    RecordProperty("native_matched", std::to_string(matched));
    RecordProperty("native_unmatched", std::to_string(native.selected.size()-matched));
    RecordProperty("native_selected_force_patch_rejections", std::to_string(singular));
}
#endif
} // namespace crash::modelio::tied_shell::test
