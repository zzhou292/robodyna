#include "ActualSupport.h"
#include <set>

namespace crash::modelio::solid_source::test {
namespace {
constexpr auto PolicyV5 = Policy::OriginalVehicleSupportsV5;
const VehicleSolidSource& Supports() {
    static const auto source = VehicleSolidSource::Prepare(vehicle::test::Canonical(),
        MemberBytes(), PolicyV5, Limits::ExtendedSolids());
    return source;
}
const Part& AirbagPart(const Data& data) {
    const auto found = std::find_if(data.parts.begin(), data.parts.end(),
                                  [](const Part& part) { return part.id == 2000945; });
    output::Require(found != data.parts.end(), "Missing airbag source material");
    return *found;
}
}
TEST(VehicleAirbagSolidSource, All80OriginalCellsAnd156NodesUseExplicitSelectedReferences) {
    const auto& source = Supports();
    const auto& data = source.data();
    EXPECT_EQ(data.parts.size(), 17u);
    EXPECT_EQ(data.rows.size(), 4980u);
    EXPECT_EQ(data.outside_solids, 10254u);
    EXPECT_EQ(data.solid18_law44.size(), 386u);
    const auto& material = AirbagPart(data);
    EXPECT_EQ(material.converter_isolid, 5u);
    EXPECT_EQ(material.selected_isolid, 18u);
    EXPECT_EQ(material.original_ihq, 4u);
    EXPECT_EQ(material.original_qh, .02);
    EXPECT_EQ(material.law44.material.hardening, tl::material::law44::solid::HardeningKind::Analytic);
    EXPECT_EQ(material.law44.curve.count, 0u);
    EXPECT_EQ(material.law44.curve.plastic_strain, nullptr);
    EXPECT_EQ(material.law44.curve.yield_stress_pa, nullptr);
    EXPECT_EQ(output::Bits(material.law44.material.analytic.b_pa),
              output::Bits((10. * 1000. / (1000. - 10.)) * 1e6));
    EXPECT_EQ(output::Bits(material.density_kg_m3), output::Bits(1.95e-9 * 1e12));
    const auto& global = data.sources.at(material.hourglass_source);
    EXPECT_EQ(global.block.filename, "combine.key");
    EXPECT_EQ(output::Sha256(global.block.raw_text), detail::AirbagHourglassHash);
    const auto ids = detail::Decode<std::uint64_t>(source.canonical().data(), "node_ids");
    const auto positions = detail::Decode<double>(source.canonical().data(), "node_positions");
    std::set<std::uint64_t> nodes, repeated;
    std::vector<std::uint64_t> records;
    unsigned count = 0;
    for (const auto& row : data.rows) {
        if (row.part_id != detail::AirbagPart) continue;
        ++count;
        ASSERT_EQ(row.family, Family::Solid18Law44);
        records.insert(records.end(), {row.element_id, row.part_id});
        records.insert(records.end(), row.raw_node_ids.begin(), row.raw_node_ids.end());
        const auto& ref = data.solid18_law44.at(row.reference_index);
        ASSERT_TRUE(ref.prepared());
        EXPECT_EQ(ref.input().source_element_id, row.element_id);
        EXPECT_EQ(ref.input().source_part_id, detail::AirbagPart);
        EXPECT_EQ(ref.input().source_section_id, detail::AirbagPart);
        EXPECT_EQ(ref.input().source_material_id, detail::AirbagPart);
        EXPECT_EQ(output::Bits(ref.input().density_kg_m3), output::Bits(material.density_kg_m3));
        EXPECT_EQ(ref.input().profile.pressure, 1u);
        EXPECT_EQ(ref.input().profile.small_strain, 2u);
        if (ref.topology() == tl::fea::solid18::law44::SourceTopology::RepeatedPairs56And78)
            repeated.insert(row.element_id);
        for (unsigned slot = 0; slot < 8; ++slot) {
            const auto node = row.canonical_nodes[slot];
            nodes.insert(row.raw_node_ids[slot]);
            EXPECT_EQ(ref.input().source_node_id[slot], row.raw_node_ids[slot]);
            EXPECT_EQ(ref.input().source_node_id[slot], ids.at(node));
            const auto p = ref.input().position_m[slot];
            EXPECT_EQ(output::Bits(p.x), output::Bits(positions.at(3 * node)));
            EXPECT_EQ(output::Bits(p.y), output::Bits(positions.at(3 * node + 1)));
            EXPECT_EQ(output::Bits(p.z), output::Bits(positions.at(3 * node + 2)));
            EXPECT_GT(ref.mass().source_nodal_mass_kg[slot], 0);
            EXPECT_GT(ref.geometry().point[slot].initial_volume_m3, 0);
        }
    }
    const output::arrays::Layout layout{output::arrays::Scalar::UInt64, count, 10, {}};
    const auto bytes = output::arrays::Encode(layout, records.data(), records.size());
    EXPECT_EQ(output::Sha256(bytes), "5e7642afe341944fff6f6bbe8c3ab5e05310f48dc387bbe2d3dfe888117c43d0");
    EXPECT_EQ(count, 80u);
    EXPECT_EQ(nodes.size(), 156u);
    EXPECT_EQ(repeated, (std::set<std::uint64_t>{2167690, 2167709, 2167737, 2167756}));
    RecordProperty("selected_solids", data.rows.size());
    RecordProperty("airbag_cells", count);
    RecordProperty("airbag_nodes", nodes.size());
    RecordProperty("forecast_bytes", source.forecast().total_bytes);
    RecordProperty("owned_payload_bytes", data.owned_payload_bytes);
}
TEST(VehicleAirbagSolidSource, Existing4900RowsCurvesAndRepresentedReferencesStayUnchanged) {
    const auto previous = VehicleSolidSource::Prepare(vehicle::test::Canonical(), MemberBytes(),
        Policy::OriginalExtendedSolidsV4, Limits::ExtendedSolids());
    const auto& before = previous.data();
    const auto& next = Supports().data();
    std::size_t cursor = 0;
    for (const auto& row : next.rows) {
        if (row.part_id == detail::AirbagPart) continue;
        ASSERT_LT(cursor, before.rows.size());
        const auto& prior = before.rows[cursor++];
        EXPECT_EQ(row.element_id, prior.element_id);
        EXPECT_EQ(row.part_id, prior.part_id);
        EXPECT_EQ(row.raw_card, prior.raw_card);
        EXPECT_EQ(row.raw_node_ids, prior.raw_node_ids);
        EXPECT_EQ(row.canonical_nodes, prior.canonical_nodes);
        EXPECT_EQ(row.family, prior.family);
        EXPECT_EQ(row.six_to_raw, prior.six_to_raw);
        if (row.family == Family::Solid18Law44) {
            const auto& a = next.solid18_law44.at(row.reference_index);
            const auto& b = before.solid18_law44.at(prior.reference_index);
            InputBits(a.input(), b.input(), 8);
            EXPECT_EQ(output::Bits(a.mass().element_mass_kg), output::Bits(b.mass().element_mass_kg));
        } else {
            EXPECT_EQ(row.reference_index, prior.reference_index);
        }
    }
    EXPECT_EQ(cursor, 4900u);
    EqualBits(next.plastic_strain, before.plastic_strain);
    EqualBits(next.yield_stress_pa, before.yield_stress_pa);
    EqualBits(next.rear_plastic_strain, before.rear_plastic_strain);
    EqualBits(next.rear_yield_stress_pa, before.rear_yield_stress_pa);
    EqualBits(next.foam_compression_strain, before.foam_compression_strain);
    EqualBits(next.foam_curve_ordinate, before.foam_curve_ordinate);
}
TEST(VehicleAirbagSolidSource, ExactCapsAndBorrowedMemberLifetimeKeepPublicationTransactional) {
    const auto& canonical = vehicle::test::Canonical();
    auto limits = Limits::ExtendedSolids();
    limits.parents = 4979;
    EXPECT_THROW(VehicleSolidSource::Preflight(canonical, PolicyV5, limits), std::runtime_error);
    EXPECT_THROW(VehicleSolidSource::Preflight(canonical, PolicyV5), std::runtime_error);
    limits = Limits::ExtendedSolids();
    const auto forecast = VehicleSolidSource::Preflight(canonical, PolicyV5, limits);
    limits.host_bytes = forecast.total_bytes - 1;
    EXPECT_THROW(VehicleSolidSource::Preflight(canonical, PolicyV5, limits), std::runtime_error);
    ++limits.host_bytes;
    EXPECT_EQ(VehicleSolidSource::Preflight(canonical, PolicyV5, limits).total_bytes, forecast.total_bytes);
    std::string member = MemberBytes();
    member.back() ^= 1;
    EXPECT_THROW(VehicleSolidSource::Prepare(canonical, member, PolicyV5, limits), std::runtime_error);
    member.back() ^= 1;
    const auto retained = VehicleSolidSource::Prepare(canonical, member, PolicyV5, limits);
    member.clear();
    member.shrink_to_fit();
    const auto copied = retained;
    const auto& part = AirbagPart(copied.data());
    EXPECT_EQ(part.law44.material.analytic.a_pa, 20e6);
    EXPECT_EQ(part.law44.curve.count, 0u);
    EXPECT_EQ(copied.data().sources.at(part.sources[2]).block.raw_text,
              Supports().data().sources.at(AirbagPart(Supports().data()).sources[2]).block.raw_text);
    EXPECT_EQ(&copied.data(), &retained.data());
}
} // namespace crash::modelio::solid_source::test
