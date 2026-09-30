#include "ActualSupport.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include "lib_utest/qualification/solid18_reference/SourceFixture.h"
#include "lib_utest/qualification/solid24_reference/SourceFixture.h"
#include "lib_utest/qualification/solid24_reference/JacobianSupport.h"
#include "lib_utest/qualification/solid6z_reference/SourceFixture.h"
#include "lib_utest/qualification/solid_law36_point/source_fixture/OriginalMaterial.h"
#include <map>
#include <iostream>

namespace crash::modelio::solid_source::test {
TEST(VehicleSolidSource, Complete2412CellsPreserveEveryQualifiedReferenceAndMaterialInput) {
    const auto& original = Original();
    const auto& data = original.data();
    EXPECT_EQ(&original.canonical().data(), &vehicle::test::Canonical().data());
    ASSERT_EQ(data.rows.size(), 2412u);
    ASSERT_EQ(data.parts.size(), 9u);
    EXPECT_EQ(data.original_solids, 15234u);
    EXPECT_EQ(data.outside_solids, 12822u);
    ASSERT_EQ(data.solid18.size(), solid18_test::SourceCount);
    ASSERT_EQ(data.solid24.size(), 1309u);
    ASSERT_EQ(data.solid6z.size(), 195u);
    for (unsigned e = 0; e < solid18_test::SourceCount; ++e) {
        const auto input = solid18_test::Source(e);
        SCOPED_TRACE(input.source_element_id);
        InputBits(data.solid18[e].input(), input, 8);
        tl::fea::solid18::Reference expected;
        ASSERT_EQ(tl::fea::solid18::InitializeReference(input, expected), tl::fea::solid18::Status::Success);
        EqualBits(solid18_test::Values(data.solid18[e]), solid18_test::Values(expected));
    }
    unsigned brick = 0, wedge = 0;
    for (unsigned e = 0; e < solid24_test::SourceCount; ++e) {
        auto input = solid24_test::Source(e);
        SCOPED_TRACE(input.source_element_id);
        if (solid24_test::IsBrick(input)) {
            input.profile.reference_strain = tl::fea::solid24::ReferenceStrain::TotalLagrangian10;
            input.profile.working_length = tl::fea::solid24::WorkingLengthUnit::Millimetre;
            const auto& actual = data.solid24.at(brick++);
            InputBits(actual.input(), input, 8);
            tl::fea::solid24::Reference expected;
            ASSERT_EQ(tl::fea::solid24::InitializeReference(input, expected), tl::fea::solid24::Status::Success);
            EqualBits(solid24_test::Values(actual), solid24_test::Values(expected));
            ASSERT_NE(actual.reference_jacobian(), nullptr);
            for (unsigned k = 0; k < 9; ++k)
                EXPECT_EQ(output::Bits(actual.reference_jacobian()->inverse[k]),
                          output::Bits(expected.reference_jacobian()->inverse[k]));
        } else {
            tl::fea::solid6z::ReferenceInput selected;
            ASSERT_TRUE(solid6z_test::Source(e, selected));
            const auto& actual = data.solid6z.at(wedge++);
            InputBits(actual.input(), selected, 6);
            tl::fea::solid6z::Reference expected;
            ASSERT_EQ(tl::fea::solid6z::InitializeReference(selected, expected), tl::fea::solid6z::Status::Success);
            EqualBits(solid6z_test::Values(actual), solid6z_test::Values(expected));
        }
    }
    EXPECT_EQ(brick, 1309u);
    EXPECT_EQ(wedge, 195u);
    const auto& adhesive = data.parts.back().law36;
    EXPECT_EQ(output::Bits(adhesive.young_pa), output::Bits(law36_test::E));
    EXPECT_EQ(output::Bits(adhesive.poisson_ratio), output::Bits(law36_test::Nu));
    EXPECT_EQ(output::Bits(adhesive.density_kg_m3), output::Bits(law36_test::Rho));
    ASSERT_EQ(adhesive.curve.count, law36_test::Curve.count);
    for (unsigned i = 0; i < adhesive.curve.count; ++i) {
        EXPECT_EQ(output::Bits(adhesive.curve.plastic_strain[i]), output::Bits(law36_test::Curve.plastic_strain[i]));
        EXPECT_EQ(output::Bits(adhesive.curve.yield_stress_pa[i]), output::Bits(law36_test::Curve.yield_stress_pa[i]));
    }
    RecordProperty("complete_source_forecast_bytes", original.forecast().total_bytes);
    RecordProperty("source_owned_payload_bytes", data.owned_payload_bytes);
    RecordProperty("selected_physical_nodes", data.canonical_nodes.size());
}
TEST(VehicleSolidSource, SourceRowsRawEightSlotsTypedMappingsAndOmittedScopeStayComplete) {
    const auto& data = Original().data();
    std::set<std::uint64_t> ids;
    std::uint32_t previous = 0;
    for (const auto& row : data.rows) {
        SCOPED_TRACE(row.element_id);
        EXPECT_TRUE(ids.insert(row.element_id).second);
        EXPECT_GT(row.source_line, previous);
        previous = row.source_line;
        ASSERT_LT(row.part_index, data.parts.size());
        EXPECT_EQ(data.parts[row.part_index].id, row.part_id);
        EXPECT_EQ(assembly::reader::auxiliary::Id(row.raw_card, 0, 8), row.element_id);
        for (unsigned slot = 0; slot < 8; ++slot)
            EXPECT_EQ(assembly::reader::auxiliary::Id(row.raw_card, 16 + 8 * slot, 8), row.raw_node_ids[slot]);
        if (row.family == Family::Solid6z)
            EXPECT_EQ(row.six_to_raw, (std::array<std::uint8_t, 6>{0, 1, 4, 3, 2, 6}));
    }
    EXPECT_EQ(data.outside_solids + data.rows.size(), data.original_solids);
    auto retained = Original();
    EXPECT_EQ(&retained.data(), &Original().data());
    EXPECT_EQ(retained.data().parts.back().law36.curve.plastic_strain, data.plastic_strain.data());
}
TEST(VehicleSolidSource, LateByteIdentityAndCountFailuresPreservePublishedSourceThenRetry) {
    const auto& before = Original();
    const auto* rows = before.data().rows.data();
    auto limits = Limits{};
    limits.host_bytes = before.forecast().total_bytes - 1;
    EXPECT_THROW(VehicleSolidSource::Prepare(before.canonical(), MemberBytes(), before.data().policy, limits), std::runtime_error);
    ++limits.host_bytes;
    const auto retry = VehicleSolidSource::Prepare(before.canonical(), MemberBytes(), before.data().policy, limits);
    EXPECT_EQ(retry.forecast().total_bytes, limits.host_bytes);
    EXPECT_EQ(before.data().rows.data(), rows);
    EXPECT_EQ(retry.data().rows.back().element_id, before.data().rows.back().element_id);
    limits = {};
    limits.parents = 2411;
    EXPECT_THROW(VehicleSolidSource::Prepare(before.canonical(), "not read after count rejection",
        before.data().policy, limits), std::runtime_error);
    auto changed = MemberBytes();
    changed.back() ^= 1;
    EXPECT_THROW(VehicleSolidSource::Prepare(before.canonical(), changed, before.data().policy), std::runtime_error);
    EXPECT_EQ(before.data().rows.data(), rows);
}
} // namespace crash::modelio::solid_source::test
