#include "ActualSupport.h"
#include "ExtendedReceipt.h"
#include "lib_utest/qualification/solid18_reference/TestSupport.h"
#include "lib_utest/qualification/solid24_reference/JacobianSupport.h"
#include "lib_utest/qualification/solid6z_reference/TestSupport.h"
#include <map>

namespace crash::modelio::solid_source::test {
namespace {
template<class Reference>
void SourceInput(const Reference& reference, const Row& row, const Part& part,
                 const std::vector<std::uint64_t>& nodes, const std::vector<double>& positions,
                 unsigned slots) {
    ASSERT_TRUE(reference.prepared());
    const auto& input = reference.input();
    EXPECT_EQ(input.source_element_id, row.element_id);
    EXPECT_EQ(input.source_part_id, row.part_id);
    EXPECT_EQ(input.source_section_id, part.section_id);
    EXPECT_EQ(input.source_material_id, part.material_id);
    EXPECT_EQ(output::Bits(input.density_kg_m3), output::Bits(part.density_kg_m3));
    for (unsigned n = 0; n < slots; ++n) {
        const unsigned raw = slots == 6 ? row.six_to_raw[n] : n;
        const auto node = row.canonical_nodes[raw];
        ASSERT_LT(node, nodes.size());
        EXPECT_EQ(input.source_node_id[n], row.raw_node_ids[raw]);
        EXPECT_EQ(input.source_node_id[n], nodes[node]);
        EXPECT_EQ(output::Bits(input.position_m[n].x), output::Bits(positions[3 * node]));
        EXPECT_EQ(output::Bits(input.position_m[n].y), output::Bits(positions[3 * node + 1]));
        EXPECT_EQ(output::Bits(input.position_m[n].z), output::Bits(positions[3 * node + 2]));
        EXPECT_GT(reference.mass().source_slot_mass_kg[n], 0);
    }
    EXPECT_GT(reference.mass().element_mass_kg, 0);
}
}
TEST(VehicleSolidExtendedSource, CompleteFourPart837CellReceiptGeometryAndMaterialUnits) {
    const auto& source = Extended();
    const auto& data = source.data();
    ASSERT_EQ(data.parts.size(), 13u);
    ASSERT_EQ(data.rows.size(), 3249u);
    EXPECT_EQ(data.solid18.size(), 908u);
    EXPECT_EQ(data.solid24.size(), 1991u);
    EXPECT_EQ(data.solid6z.size(), 350u);
    EXPECT_EQ(data.original_solids, 15234u);
    EXPECT_EQ(data.outside_solids, 11985u);
    EXPECT_EQ(&source.canonical().data(), &vehicle::test::Canonical().data());
    const auto& canonical = source.canonical().data();
    const auto nodes = detail::Decode<std::uint64_t>(canonical, "node_ids");
    const auto positions = detail::Decode<double>(canonical, "node_positions");
    std::map<std::uint64_t, std::vector<std::uint64_t>> packets;
    std::map<std::uint64_t, std::array<std::size_t, 2>> counts;
    std::set<std::uint32_t> used_nodes;
    std::uint32_t previous_line = 0;
    for (const auto& row : data.rows) {
        SCOPED_TRACE(row.element_id);
        EXPECT_GT(row.source_line, previous_line);
        previous_line = row.source_line;
        used_nodes.insert(row.canonical_nodes.begin(), row.canonical_nodes.end());
        const auto* receipt = Added(row.part_id);
        if (!receipt) continue;
        ASSERT_LT(row.part_index, data.parts.size());
        const auto& part = data.parts[row.part_index];
        EXPECT_EQ(part.id, receipt->pid);
        EXPECT_EQ(part.material_law, MaterialLaw::Law42);
        EXPECT_EQ(part.converter_isolid, 1u);
        EXPECT_EQ(output::Bits(part.density_kg_m3), output::Bits(receipt->source_density * 1e12));
        EXPECT_EQ(output::Bits(part.law42.density_kg_m3), output::Bits(part.density_kg_m3));
        EXPECT_EQ(part.law42.mu_pa, 24. * 1e6);
        EXPECT_EQ(part.law42.poisson_ratio, .463);
        EXPECT_EQ(part.law42.tension_cutoff_pa, 1e20 * 1e6);
        ASSERT_LT(part.hourglass_source, data.sources.size());
        EXPECT_EQ(part.hourglass_id, 2000017u);
        const auto& section = data.sources.at(part.sources[1]);
        EXPECT_FALSE(vehicle::detail::SourceScalar(section.cards[0].second, 1));
        auto& packet = packets[row.part_id];
        packet.insert(packet.end(), {row.element_id, row.part_id});
        packet.insert(packet.end(), row.raw_node_ids.begin(), row.raw_node_ids.end());
        if (row.family == Family::Solid24) {
            SourceInput(data.solid24.at(row.reference_index), row, part, nodes, positions, 8);
            const auto& input = data.solid24.at(row.reference_index).input();
            EXPECT_EQ(input.profile.reference_strain, tl::fea::solid24::ReferenceStrain::TotalLagrangian10);
            EXPECT_EQ(input.profile.working_length, tl::fea::solid24::WorkingLengthUnit::Millimetre);
            ASSERT_NE(data.solid24.at(row.reference_index).reference_jacobian(), nullptr);
            ++counts[row.part_id][0];
        } else {
            ASSERT_EQ(row.family, Family::Solid6z);
            EXPECT_EQ(row.six_to_raw, (std::array<std::uint8_t, 6>{0, 1, 4, 3, 2, 6}));
            SourceInput(data.solid6z.at(row.reference_index), row, part, nodes, positions, 6);
            ++counts[row.part_id][1];
        }
    }
    EXPECT_EQ(std::vector<std::uint32_t>(used_nodes.begin(), used_nodes.end()), data.canonical_nodes);
    for (const auto& receipt : AddedRubber) {
        EXPECT_EQ(counts[receipt.pid], (std::array<std::size_t, 2>{receipt.bricks, receipt.wedges}));
        const auto& packet = packets[receipt.pid];
        const output::arrays::Layout layout{output::arrays::Scalar::UInt64, packet.size() / 10, 10, {}};
        const auto bytes = output::arrays::Encode(layout, packet.data(), packet.size());
        EXPECT_EQ(output::Sha256(bytes), receipt.ordered_records_sha256);
    }
    RecordProperty("complete_source_forecast_bytes", source.forecast().total_bytes);
    RecordProperty("source_owned_payload_bytes", data.owned_payload_bytes);
    RecordProperty("selected_physical_nodes", data.canonical_nodes.size());
    RecordProperty("added_source_cells", 837);
}
TEST(VehicleSolidExtendedSource, EveryOld2412ReferenceAndCurveRemainBitExactInSourceOrder) {
    const auto& before = Original().data();
    const auto& after = Extended().data();
    std::size_t old = 0;
    for (const auto& row : after.rows) {
        if (Added(row.part_id)) continue;
        ASSERT_LT(old, before.rows.size());
        const auto& prior = before.rows[old++];
        SCOPED_TRACE(row.element_id);
        EXPECT_EQ(row.element_id, prior.element_id);
        EXPECT_EQ(row.part_id, prior.part_id);
        EXPECT_EQ(row.canonical_row, prior.canonical_row);
        EXPECT_EQ(row.source_line, prior.source_line);
        EXPECT_EQ(row.raw_card, prior.raw_card);
        EXPECT_EQ(row.raw_node_ids, prior.raw_node_ids);
        EXPECT_EQ(row.canonical_nodes, prior.canonical_nodes);
        EXPECT_EQ(row.family, prior.family);
        if (row.family == Family::Solid18) {
            const auto& a = after.solid18.at(row.reference_index);
            const auto& b = before.solid18.at(prior.reference_index);
            InputBits(a.input(), b.input(), 8);
            EqualBits(solid18_test::Values(a), solid18_test::Values(b));
        } else if (row.family == Family::Solid24) {
            const auto& a = after.solid24.at(row.reference_index);
            const auto& b = before.solid24.at(prior.reference_index);
            InputBits(a.input(), b.input(), 8);
            EqualBits(solid24_test::Values(a), solid24_test::Values(b));
            EqualBits(solid24_test::JacobianValues(a), solid24_test::JacobianValues(b));
        } else {
            const auto& a = after.solid6z.at(row.reference_index);
            const auto& b = before.solid6z.at(prior.reference_index);
            InputBits(a.input(), b.input(), 6);
            EqualBits(solid6z_test::Values(a), solid6z_test::Values(b));
        }
    }
    EXPECT_EQ(old, 2412u);
    EqualBits(after.plastic_strain, before.plastic_strain);
    EqualBits(after.yield_stress_pa, before.yield_stress_pa);
    EXPECT_EQ(before.policy, Policy::OriginalAdhesive18RubberHephS6zV1);
    // Both default forecasts reserve the same4096-parent capacity. V2 fits the
    // existing reservation; its actual owned payload is reported separately.
    EXPECT_EQ(Extended().forecast().total_bytes, Original().forecast().total_bytes);
}
TEST(VehicleSolidExtendedSource, ExactCapsLateSourceFailureAndRetryPreserveBothHandles) {
    const auto& before = Extended();
    const auto* retained_rows = before.data().rows.data();
    auto limits = Limits{};
    limits.host_bytes = before.forecast().total_bytes - 1;
    EXPECT_THROW(VehicleSolidSource::Prepare(before.canonical(), "unread", before.data().policy, limits), std::runtime_error);
    ++limits.host_bytes;
    const auto retry = VehicleSolidSource::Prepare(before.canonical(), MemberBytes(), before.data().policy, limits);
    EXPECT_EQ(retry.forecast().total_bytes, limits.host_bytes);
    EXPECT_EQ(retry.data().rows.back().element_id, before.data().rows.back().element_id);
    limits = {};
    limits.parents = 3248;
    EXPECT_THROW(VehicleSolidSource::Prepare(before.canonical(), "unread", before.data().policy, limits), std::runtime_error);
    ++limits.parents;
    EXPECT_NO_THROW(VehicleSolidSource::Preflight(before.canonical(), before.data().policy, limits));
    const auto exact_count = VehicleSolidSource::Prepare(before.canonical(), MemberBytes(), before.data().policy, limits);
    EXPECT_EQ(exact_count.data().rows.size(), limits.parents);
    auto changed = MemberBytes();
    changed.back() ^= 1;
    EXPECT_THROW(VehicleSolidSource::Prepare(before.canonical(), changed, before.data().policy), std::runtime_error);
    EXPECT_EQ(before.data().rows.data(), retained_rows);
    EXPECT_EQ(Original().data().rows.size(), 2412u);
    auto copy = before;
    EXPECT_EQ(&copy.data(), &before.data());
    EXPECT_EQ(copy.data().parts.back().law36.curve.plastic_strain, before.data().plastic_strain.data());
}
} // namespace crash::modelio::solid_source::test
