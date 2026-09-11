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
    Data data = Rubber();
    std::string member;
    Geometry(bool wedge) {
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
        std::vector<std::uint64_t> records{100, 2000477};
        std::ostringstream card;
        card << std::setw(8) << 100 << std::setw(8) << 2000477;
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
} // namespace crash::modelio::solid_source::test
