#include "ActualSupport.h"
#include "lib_utest/qualification/law90_preparation/TestSupport.h"

namespace crash::modelio::solid_source::test {
namespace {
constexpr auto RadiatorPolicy = Policy::OriginalExtendedSolidsV4;
const VehicleSolidSource& AllSolids() {
    static const auto result = VehicleSolidSource::Prepare(vehicle::test::Canonical(),
        MemberBytes(), RadiatorPolicy, Limits::ExtendedSolids());
    return result;
}
output::Document NativeObservation() {
    const auto* path = std::getenv("ROBO_LAW90_NATIVE_OBSERVATION");
    output::Require(path && *path, "Missing native SDI observation");
    const auto bytes = output::ReadBounded(path, 32768);
    output::Require(output::Sha256(bytes) ==
        "662fba9b45aceba5da1563574f80669adca99b114f3fe918d41e1a82fd06dd88",
        "Qualified original native SDI observation changed");
    output::Document result;
    result.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(), bytes.size());
    output::Require(!result.HasParseError(), "Invalid native SDI observation");
    return result;
}
}
TEST(VehicleRadiatorSolidSource, ActualDefaultsCurveAndAll1345ReferencesMatchNativeSource) {
    const auto& source = AllSolids();
    const auto& data = source.data();
    EXPECT_EQ(data.parts.size(), 16u);
    EXPECT_EQ(data.rows.size(), 4900u);
    EXPECT_EQ(data.outside_solids, 10334u);
    ASSERT_EQ(data.solid18_law90.size(), 1345u);
    const auto found = std::find_if(data.parts.begin(), data.parts.end(),
                                  [](const Part& part) { return part.id == 2000063; });
    ASSERT_NE(found, data.parts.end());
    const auto native = NativeObservation();
    const auto& expected = native["direct"]["native_preparation_si"];
    double values[33]; law90_test::Pack(found->law90, values);
    ASSERT_EQ(expected.Size(), 33u);
    for (unsigned i = 0; i < 33; ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(output::Bits(values[i]), output::Bits(expected[i].GetDouble()));
    }
    EXPECT_EQ(found->law90.reader().loading_flag, 1);
    EXPECT_EQ(found->law90_input.hysteresis, 0);
    EXPECT_EQ(found->law90.reader().curve_scale, 1e6);
    const auto& curve = native["direct"]["curve_working_xy"];
    ASSERT_EQ(data.foam_compression_strain.size(), 28u);
    ASSERT_EQ(curve.Size(), 56u);
    for (unsigned i = 0; i < 28; ++i) {
        EXPECT_EQ(output::Bits(data.foam_compression_strain[i]), output::Bits(curve[2*i].GetDouble()));
        EXPECT_EQ(output::Bits(data.foam_curve_ordinate[i]), output::Bits(curve[2*i+1].GetDouble()));
    }
    const auto ids = detail::Decode<std::uint64_t>(source.canonical().data(), "node_ids");
    const auto positions = detail::Decode<double>(source.canonical().data(), "node_positions");
    unsigned count = 0;
    for (const auto& row : data.rows) {
        if (row.family != Family::Solid18Law90) continue;
        ++count;
        const auto& ref = data.solid18_law90.at(row.reference_index);
        ASSERT_TRUE(ref.prepared());
        EXPECT_EQ(ref.input().source_element_id, row.element_id);
        EXPECT_EQ(ref.input().source_part_id, 2000063u);
        EXPECT_EQ(ref.input().density_kg_m3, found->density_kg_m3);
        EXPECT_EQ(ref.input().profile.pressure, 0u);
        EXPECT_EQ(ref.input().profile.small_strain, 10u);
        for (unsigned slot = 0; slot < 8; ++slot) {
            const auto n = row.canonical_nodes[slot];
            EXPECT_EQ(ref.input().source_node_id[slot], ids.at(n));
            EXPECT_EQ(ref.input().source_node_id[slot], row.raw_node_ids[slot]);
            const auto x = ref.input().position_m[slot];
            EXPECT_EQ(output::Bits(x.x), output::Bits(positions.at(3*n)));
            EXPECT_EQ(output::Bits(x.y), output::Bits(positions.at(3*n+1)));
            EXPECT_EQ(output::Bits(x.z), output::Bits(positions.at(3*n+2)));
            EXPECT_GT(ref.mass().source_nodal_mass_kg[slot], 0);
        }
    }
    EXPECT_EQ(count, 1345u);
    RecordProperty("selected_solids", data.rows.size());
    RecordProperty("forecast_bytes", source.forecast().total_bytes);
    RecordProperty("owned_payload_bytes", data.owned_payload_bytes);
}
TEST(VehicleRadiatorSolidSource, WrongOriginalUnloadingBranchAndCapsReject) {
    auto data = AllSolids().data();
    auto part = *std::find_if(data.parts.begin(), data.parts.end(),
                             [](const Part& value) { return value.id == 2000063; });
    auto evidence = data.sources.at(part.sources[2]);
    evidence.cards[0].second.resize(80, ' ');
    evidence.cards[0].second.replace(50, 10, "         1");
    EXPECT_THROW(detail::ReadRadiatorMaterial(part, evidence, data), std::runtime_error);
    auto limits = Limits::ExtendedSolids();
    limits.parents = 4899;
    EXPECT_THROW(VehicleSolidSource::Preflight(vehicle::test::Canonical(), RadiatorPolicy, limits), std::runtime_error);
    EXPECT_THROW(VehicleSolidSource::Preflight(vehicle::test::Canonical(), RadiatorPolicy), std::runtime_error);
    EXPECT_THROW(VehicleSolidSource::Preflight(vehicle::test::Canonical(),
        Policy::OriginalAdhesive18RubberHephS6zV1, Limits::ExtendedSolids()), std::runtime_error);
    limits = Limits::ExtendedSolids();
    const auto forecast = VehicleSolidSource::Preflight(vehicle::test::Canonical(), RadiatorPolicy, limits);
    limits.host_bytes = forecast.total_bytes - 1;
    EXPECT_THROW(VehicleSolidSource::Preflight(vehicle::test::Canonical(), RadiatorPolicy, limits), std::runtime_error);
    ++limits.host_bytes;
    EXPECT_EQ(VehicleSolidSource::Preflight(vehicle::test::Canonical(), RadiatorPolicy, limits).total_bytes,
              forecast.total_bytes);
}
TEST(VehicleRadiatorSolidSource, Earlier3555SourceRecordsAndOwnedCurvesStayUnchanged) {
    const auto rear = VehicleSolidSource::Prepare(vehicle::test::Canonical(), MemberBytes(),
        Policy::OriginalAdhesive18ExtendedRubberRearLaw44V3);
    const auto& previous = rear.data();
    const auto& next = AllSolids().data();
    std::size_t cursor = 0;
    for (const auto& row : next.rows) {
        if (row.family == Family::Solid18Law90) continue;
        ASSERT_LT(cursor, previous.rows.size());
        const auto& prior = previous.rows[cursor++];
        EXPECT_EQ(row.element_id, prior.element_id);
        EXPECT_EQ(row.part_id, prior.part_id);
        EXPECT_EQ(row.raw_card, prior.raw_card);
        EXPECT_EQ(row.raw_node_ids, prior.raw_node_ids);
        EXPECT_EQ(row.canonical_nodes, prior.canonical_nodes);
        EXPECT_EQ(row.six_to_raw, prior.six_to_raw);
        EXPECT_EQ(row.family, prior.family);
        EXPECT_EQ(row.reference_index, prior.reference_index);
        const auto& a = next.parts.at(row.part_index);
        const auto& b = previous.parts.at(prior.part_index);
        EXPECT_EQ(a.id, b.id);
        EXPECT_EQ(a.material_law, b.material_law);
        EXPECT_EQ(output::Bits(a.density_kg_m3), output::Bits(b.density_kg_m3));
    }
    EXPECT_EQ(cursor, 3555u);
    EqualBits(next.plastic_strain, previous.plastic_strain);
    EqualBits(next.yield_stress_pa, previous.yield_stress_pa);
    EqualBits(next.rear_plastic_strain, previous.rear_plastic_strain);
    EqualBits(next.rear_yield_stress_pa, previous.rear_yield_stress_pa);
}
} // namespace crash::modelio::solid_source::test
