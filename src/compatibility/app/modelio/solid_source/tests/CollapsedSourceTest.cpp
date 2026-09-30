#include "ActualSupport.h"
#include <iomanip>
#include "lib_utest/qualification/solid24_reference/NativeOracle.h"

namespace crash::modelio::solid_source::test {
namespace {
std::string Decimal(double value) {
    std::ostringstream text;text << std::setprecision(17) << value;return text.str();
}
const VehicleSolidSource& Converted() {
    static const auto source = VehicleSolidSource::Prepare(vehicle::test::Canonical(), MemberBytes(),
        Policy::NativeConvertedSupportsV6, Limits::ExtendedSolids());
    return source;
}
}
TEST(VehicleCollapsedSolidSource, CompleteSelectionAndEveryRawSlotStaySourceBound) {
    const auto previous = VehicleSolidSource::Prepare(vehicle::test::Canonical(), MemberBytes(),
        Policy::OriginalVehicleSupportsV5, Limits::ExtendedSolids());
    const auto& before = previous.data();
    const auto& after = Converted().data();
    ASSERT_EQ(after.rows.size(), 4980u);ASSERT_EQ(after.parts.size(), 17u);
    EXPECT_EQ(after.solid24.size(), 2341u);EXPECT_TRUE(after.solid6z.empty());
    EXPECT_EQ(after.solid18.size(), before.solid18.size());
    EXPECT_EQ(after.solid18_law44.size(), before.solid18_law44.size());
    EXPECT_EQ(after.solid18_law90.size(), before.solid18_law90.size());
    EXPECT_EQ(after.canonical_nodes, before.canonical_nodes);
    unsigned changed = 0;double old_mass = 0,new_mass = 0;
    for (std::size_t i = 0; i < after.rows.size(); ++i) {
        const auto& a = after.rows[i];const auto& b = before.rows[i];
        EXPECT_EQ(a.element_id, b.element_id);EXPECT_EQ(a.part_id, b.part_id);
        EXPECT_EQ(a.raw_card, b.raw_card);EXPECT_EQ(a.source_line, b.source_line);
        EXPECT_EQ(a.raw_node_ids, b.raw_node_ids);EXPECT_EQ(a.canonical_nodes, b.canonical_nodes);
        if (b.family != Family::Solid6z) {EXPECT_EQ(a.family, b.family);continue;}
        ++changed;ASSERT_EQ(a.family, Family::Solid24);
        const auto& reference = after.solid24.at(a.reference_index);
        EXPECT_EQ(reference.unique_node_count(), 6u);
        EXPECT_EQ(reference.input().profile.connectivity,
                  tl::fea::solid24::ConnectivityProfile::CollapsedTopEdges);
        for (unsigned n = 0; n < 8; ++n)
            EXPECT_EQ(reference.input().source_node_id[n], a.raw_node_ids[n]);
        old_mass += before.solid6z.at(b.reference_index).mass().element_mass_kg;
        new_mass += reference.mass().element_mass_kg;
    }
    EXPECT_EQ(changed, 350u);
    EXPECT_GT(new_mass, old_mass);
    RecordProperty("changed_formulation_cells", changed);
    RecordProperty("collapsed_heph_mass_kg", Decimal(new_mass));
    RecordProperty("prior_s6_mass_kg", Decimal(old_mass));
    RecordProperty("solid_formulation_mass_difference_kg", Decimal(new_mass-old_mass));
    RecordProperty("owned_payload_bytes", after.owned_payload_bytes);
}
TEST(VehicleCollapsedSolidSource, All2341HephReferencesMatchCompleteNativeGeometryAndMass) {
    const auto& data = Converted().data();unsigned aliases = 0;
    for (const auto& reference : data.solid24) {
        SCOPED_TRACE(reference.input().source_element_id);
        const auto native = solid24_test::Native(reference.input());
        ASSERT_EQ(native.status, 0);
        EXPECT_TRUE(solid24_test::Agree(solid24_test::Values(reference), native.values));
        for (unsigned n = 0; n < 8; ++n)
            EXPECT_EQ(reference.source_slot(n), static_cast<unsigned>(native.permutation[n]));
        if (reference.unique_node_count() == 6) ++aliases;
    }
    EXPECT_EQ(aliases, 350u);
    RecordProperty("native_heph_reference_packets", data.solid24.size());
    RecordProperty("native_collapsed_packets", aliases);
}
} // namespace crash::modelio::solid_source::test
