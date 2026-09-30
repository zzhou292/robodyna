#include "TestSupport.h"
#include "lib_utest/qualification/solid18_reference/TestSupport.h"
#include "lib_utest/qualification/solid6z_reference/TestSupport.h"
#include <numeric>

namespace crash::modelio::solid_source::test {
namespace {
template<class T> void Array(source::CanonicalData& source, unsigned slot, const char* name,
        std::size_t columns, const std::vector<T>& values) {
    auto& array = source.arrays[slot];
    array.name = name;
    output::arrays::Layout layout{output::arrays::detail::Type<T>::value, values.size() / columns, columns, {}};
    array.bytes = output::arrays::Encode(layout, values.data(), values.size());
    array.descriptor = {std::string(name) + ".bin", layout, array.bytes.size(), output::Sha256(array.bytes)};
}
struct Geometry {
    source::CanonicalData source;
    Data data;
    std::string member;
    Geometry(bool wedge, std::uint64_t id = 2000477,
             Policy policy = Policy::OriginalAdhesive18RubberHephS6zV1)
        : data(id == detail::AirbagPart ? Airbag() :
               Rubber(id, policy, id == 2000509 || id == 2000521 ? "1.9990E-9" : "1.9800E-9")) {
        Resolve(data);
        std::vector<std::uint64_t> ids;
        std::vector<double> positions;
        std::vector<std::uint32_t> nodes;
        if (wedge) {
            const auto input = solid6z_test::Wedge();
            // Invert the independently documented raw1,2,5,4,3,7 map.
            const unsigned raw[]{0, 1, 4, 3, 2, 2, 5, 5};
            for (unsigned n = 0; n < 6; ++n) {
                ids.push_back(input.source_node_id[n]);
                const auto p = input.position_m[n];
                positions.insert(positions.end(), {p.x, p.y, p.z});
            }
            nodes.assign(std::begin(raw), std::end(raw));
        } else {
            const auto input = solid18_test::Cube();
            for (unsigned n = 0; n < 8; ++n) {
                ids.push_back(input.source_node_id[n]);
                const auto p = input.position_m[n];
                positions.insert(positions.end(), {p.x, p.y, p.z});
                nodes.push_back(n);
            }
            positions[0] = -0.0;
        }
        source.canonical_nodes = ids.size();
        std::vector<std::uint64_t> records{100, id};
        std::ostringstream card;
        card << std::setw(8) << 100 << std::setw(8) << id;
        for (auto n : nodes) {
            records.push_back(ids[n]);
            card << std::setw(8) << ids[n];
        }
        member = "*ELEMENT_SOLID\n" + card.str() + "\n";
        Array(source, 0, "solids_records", 10, records);
        Array(source, 1, "solids_node_indices", 8, nodes);
        Array(source, 2, "solids_source_lines", 1, std::vector<std::uint32_t>{2});
        Array(source, 3, "solids_blank_masks", 1, std::vector<std::uint16_t>{0});
        Array(source, 4, "node_ids", 1, ids);
        Array(source, 5, "node_positions", 3, positions);
    }
    void Prepare() {
        detail::ReadGeometry(source, member, data, {});
        detail::PrepareReferences(source, data, {});
    }
};
}
TEST(VehicleSolidGeometry, NativeConvertedProfilePreservesAllEightCollapsedHephSlots) {
    Geometry fixture(true, 2000477, Policy::NativeConvertedSupportsV6);
    ASSERT_NO_THROW(fixture.Prepare());
    ASSERT_EQ(fixture.data.solid24.size(), 1u);
    EXPECT_TRUE(fixture.data.solid6z.empty());
    const auto& row = fixture.data.rows[0];
    const auto& reference = fixture.data.solid24[0];
    EXPECT_EQ(row.family, Family::Solid24);
    EXPECT_EQ(reference.unique_node_count(), 6u);
    EXPECT_EQ(reference.input().profile.connectivity,
              tl::fea::solid24::ConnectivityProfile::CollapsedTopEdges);
    double mass = 0;
    for (unsigned n = 0; n < 8; ++n) {
        EXPECT_EQ(reference.input().source_node_id[n], row.raw_node_ids[n]);
        EXPECT_GT(reference.mass().source_slot_mass_kg[n], 0);
        mass += reference.mass().source_slot_mass_kg[n];
    }
    EXPECT_DOUBLE_EQ(mass, reference.mass().element_mass_kg);
    const auto census = detail::ExpectedCensus(Policy::NativeConvertedSupportsV6);
    EXPECT_EQ(census.parents, 4980u);EXPECT_EQ(census.solid24, 2341u);EXPECT_EQ(census.solid6z, 0u);
    EXPECT_TRUE(detail::SelectedAirbag(detail::AirbagPart, Policy::NativeConvertedSupportsV6));
}
TEST(VehicleSolidGeometry, NativeConvertedDistinctHephKeepsDistinctProfile) {
    Geometry fixture(false, 2000477, Policy::NativeConvertedSupportsV6);
    ASSERT_NO_THROW(fixture.Prepare());
    ASSERT_EQ(fixture.data.solid24.size(), 1u);
    EXPECT_EQ(fixture.data.solid24[0].unique_node_count(), 8u);
    EXPECT_EQ(fixture.data.solid24[0].input().profile.connectivity,
              tl::fea::solid24::ConnectivityProfile::EightDistinct);
}
TEST(VehicleSolidGeometry, OriginalSlotsTypedWedgeMapAndNativeMassRemainExplicit) {
    Geometry fixture(true);
    ASSERT_NO_THROW(fixture.Prepare());
    ASSERT_EQ(fixture.data.solid6z.size(), 1u);
    const auto& row = fixture.data.rows[0];
    EXPECT_EQ(row.family, Family::Solid6z);
    EXPECT_EQ(row.six_to_raw, (std::array<std::uint8_t, 6>{0, 1, 4, 3, 2, 6}));
    EXPECT_EQ(row.raw_node_ids[4], row.raw_node_ids[5]);
    EXPECT_EQ(row.raw_node_ids[6], row.raw_node_ids[7]);
    const auto& reference = fixture.data.solid6z[0];
    EXPECT_EQ(reference.input().source_node_id[2], row.raw_node_ids[4]);
    double sum = 0;
    for (double mass : reference.mass().source_slot_mass_kg) sum += mass;
    EXPECT_NEAR(sum, reference.mass().element_mass_kg, 1e-15);
    EXPECT_EQ(reference.mass().isotropic_inertia_kg_m2(), 0);
}
TEST(VehicleSolidGeometry, CanonicalSignedZeroAndHephProfileReachReferenceIdentity) {
    Geometry fixture(false);
    ASSERT_NO_THROW(fixture.Prepare());
    ASSERT_EQ(fixture.data.solid24.size(), 1u);
    const auto& reference = fixture.data.solid24[0];
    EXPECT_EQ(output::Bits(reference.input().position_m[0].x), output::Bits(-0.0));
    EXPECT_EQ(reference.input().profile.working_length, tl::fea::solid24::WorkingLengthUnit::Millimetre);
    ASSERT_NE(reference.reference_jacobian(), nullptr);
    EXPECT_EQ(reference.input().source_material_id, 2000477u);
}
TEST(VehicleSolidGeometry, LateRawRowMismatchAndIncompleteWedgeRejectBeforePublication) {
    Geometry bad(true);
    const auto at = bad.member.rfind("105");
    ASSERT_NE(at, std::string::npos);
    bad.member.replace(at, 3, "999");
    EXPECT_THROW(bad.Prepare(), std::runtime_error);
    Geometry retry(true);
    ASSERT_NO_THROW(retry.Prepare());
    auto nodes = detail::Decode<std::uint32_t>(retry.source, "solids_node_indices");
    nodes.back() = 0;
    Array(retry.source, 1, "solids_node_indices", 8, nodes);
    Data next = Rubber();
    Resolve(next);
    EXPECT_THROW(detail::ReadGeometry(retry.source, retry.member, next, {}), std::runtime_error);
    EXPECT_EQ(retry.data.solid6z.size(), 1u);
}
TEST(VehicleSolidGeometry, ExtendedSourceUsesSameNativeGeometryAndCarriesAntirollDensity) {
    constexpr auto policy = Policy::OriginalAdhesive18ExtendedRubberHephS6zV2;
    for (bool wedge : {false, true}) {
        Geometry rear(wedge, 2000017, policy);
        Geometry antiroll(wedge, 2000521, policy);
        ASSERT_NO_THROW(rear.Prepare());
        ASSERT_NO_THROW(antiroll.Prepare());
        ASSERT_EQ(antiroll.data.rows.size(), 1u);
        EXPECT_EQ(antiroll.data.rows[0].part_id, 2000521u);
        const auto rear_mass = wedge ? rear.data.solid6z[0].mass().element_mass_kg :
                                      rear.data.solid24[0].mass().element_mass_kg;
        const auto antiroll_mass = wedge ? antiroll.data.solid6z[0].mass().element_mass_kg :
                                          antiroll.data.solid24[0].mass().element_mass_kg;
        EXPECT_GT(antiroll_mass, rear_mass);
        EXPECT_NEAR(antiroll_mass / rear_mass, 1.999 / 1.98, 1e-14);
        auto omitted = Rubber();
        Resolve(omitted);
        ASSERT_NO_THROW(detail::ReadGeometry(antiroll.source, antiroll.member, omitted, {}));
        EXPECT_TRUE(omitted.rows.empty());
        EXPECT_EQ(omitted.outside_solids, 1u);
    }
}
TEST(VehicleSolidGeometry, SelectedAirbagKeepsEightMassSlotsAndRejectsLateTopologyFailure) {
    for (bool repeated : {false, true}) {
        Geometry fixture(repeated, detail::AirbagPart, Policy::OriginalVehicleSupportsV5);
        ASSERT_NO_THROW(fixture.Prepare());
        ASSERT_EQ(fixture.data.solid18_law44.size(), 1u);
        const auto& ref = fixture.data.solid18_law44[0];
        EXPECT_EQ(ref.topology() == tl::fea::solid18::law44::SourceTopology::RepeatedPairs56And78, repeated);
        EXPECT_EQ(ref.input().profile.material_law, 44u);
        double mass = 0;
        for (unsigned i = 0; i < 8; ++i) {
            EXPECT_EQ(ref.input().source_node_id[i], fixture.data.rows[0].raw_node_ids[i]);
            EXPECT_GT(ref.mass().source_nodal_mass_kg[i], 0);
            mass += ref.mass().source_nodal_mass_kg[i];
            EXPECT_GT(ref.geometry().point[i].initial_volume_m3, 0);
        }
        EXPECT_NEAR(mass, ref.mass().element_mass_kg, 1e-11);
        auto bad = fixture.data;
        bad.rows.back().raw_node_ids[7] = bad.rows.back().raw_node_ids[0];
        EXPECT_THROW(detail::PrepareReferences(fixture.source, bad, {}), std::runtime_error);
        EXPECT_TRUE(ref.prepared());
        Geometry retry(repeated, detail::AirbagPart, Policy::OriginalVehicleSupportsV5);
        ASSERT_NO_THROW(retry.Prepare());
        EXPECT_EQ(output::Bits(retry.data.solid18_law44[0].mass().element_mass_kg),
                  output::Bits(ref.mass().element_mass_kg));
    }
}
} // namespace crash::modelio::solid_source::test
