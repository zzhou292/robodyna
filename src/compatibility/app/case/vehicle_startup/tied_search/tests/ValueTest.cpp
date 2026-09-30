#include "Fixture.h"
#include <cstring>
#include <type_traits>
namespace crash::cases::vehicle_startup::test {
TEST(TiedAssessmentValues, ExactWorkingInputsPreserveSourceNSVAndRepeatedT3) {
    Fixture f;
    const auto inputs = detail::Pack(f.declaration, f.packing, f.geometry);
    ASSERT_EQ(inputs.positions.size(), 12u);
    ASSERT_EQ(inputs.masters.size(), 2u);
    std::size_t triangle = 0;
    for (std::size_t n = 0; n < inputs.positions.size(); ++n)
        EXPECT_EQ(std::memcmp(&inputs.positions[n], f.geometry.working_positions[n].data(), 3*sizeof(double)), 0);
    EXPECT_TRUE(std::signbit(inputs.positions[0].x));
    EXPECT_NE(inputs.positions[1].x, (inputs.positions[1].x*.001)/.001);
    for (std::size_t rank = 0; rank < inputs.masters.size(); ++rank) {
        const auto& m = inputs.masters[rank];
        EXPECT_EQ(m.nodes, f.geometry.masters[rank].working_nodes);
        EXPECT_EQ(m.bounds_thickness, f.geometry.masters[rank].bounds_thickness);
        EXPECT_EQ(m.projection_thickness, f.geometry.masters[rank].projection_thickness);
        if (m.topology == native_search::MasterTopology::TriangleRepeatedThird) {
            ++triangle;
            EXPECT_EQ(m.nodes[2], m.nodes[3]);
        }
    }
    EXPECT_EQ(triangle, 1u);
    const auto view = inputs.View(f.geometry);
    EXPECT_EQ(view.secondary_nodes, f.geometry.secondary_working_nodes.data());
    EXPECT_EQ(view.working_length_to_m, .001);
    EXPECT_EQ(view.maximum_secondary_shell_thickness, 0);
}
TEST(TiedAssessmentValues, CompleteBudgetChargesSharedBackingOnceAndExactCapRetry) {
    Fixture f;
    const auto before = detail::Preflight(f.canonical, f.declaration, f.packing, f.geometry, {});
    EXPECT_EQ(before.driver_host_reservation_bytes, TiedAssessmentLimits{}.driver.max_host_bytes);
    EXPECT_EQ(before.input_staging_bytes, f.geometry.working_positions.size()*sizeof(native_search::Vec3) +
        f.geometry.masters.size()*sizeof(native_search::SearchMasterInput));
    EXPECT_EQ(before.total_host_bytes, before.source_payload_bytes + before.input_staging_bytes +
        before.fixed_bytes + before.driver_host_reservation_bytes);
    TiedAssessmentLimits limits;
    limits.host_bytes = before.total_host_bytes-1;
    EXPECT_THROW(detail::Preflight(f.canonical, f.declaration, f.packing, f.geometry, limits), std::exception);
    limits.host_bytes++;
    EXPECT_EQ(detail::Preflight(f.canonical, f.declaration, f.packing, f.geometry, limits).total_host_bytes,
              before.total_host_bytes);
    const auto old = f.canonical.arrays.back().bytes.capacity();
    f.canonical.arrays.back().bytes.reserve(old+1000);
    EXPECT_GT(detail::Preflight(f.canonical, f.declaration, f.packing, f.geometry, {}).source_payload_bytes,
              before.source_payload_bytes);
}
TEST(TiedAssessmentValues, LateSourceAssociationsRejectWithoutChangingPriorInputs) {
    static_assert(!std::is_copy_assignable_v<TiedSearchAssessment>);
    Fixture f;
    const auto accepted = detail::Pack(f.declaration, f.packing, f.geometry);
    const auto last = f.geometry.secondary_working_nodes.back();
    f.geometry.secondary_working_nodes.back() = UINT32_MAX;
    EXPECT_THROW(detail::Pack(f.declaration, f.packing, f.geometry), std::exception);
    EXPECT_EQ(accepted.positions.size(), 12u);
    f.geometry.secondary_working_nodes.back() = last;
    auto row = f.geometry.masters.back().declaration_row;
    f.geometry.masters.back().declaration_row = UINT32_MAX;
    EXPECT_THROW(detail::Pack(f.declaration, f.packing, f.geometry), std::exception);
    f.geometry.masters.back().declaration_row = row;
    const auto retry = detail::Pack(f.declaration, f.packing, f.geometry);
    ASSERT_EQ(retry.positions.size(), accepted.positions.size());
    EXPECT_EQ(std::memcmp(retry.positions.data(), accepted.positions.data(), accepted.positions.size()*sizeof(native_search::Vec3)), 0);
    EXPECT_EQ(retry.masters.back().nodes, accepted.masters.back().nodes);
}
TEST(TiedAssessmentValues, SelectedDeclaredIdentitySurvivesUnmatchedAndSingularRows) {
    Fixture f;
    native_search::SearchDriverResult result;
    result.rows.resize(f.declaration.slave_nodes.size());
    result.rows.back().choice.matched = true;
    result.rows.back().choice.ordered_master = 2;
    result.rows.back().force_patch_status = native_search::Status::SingularPatch;
    result.matched_count = 1;
    result.singular_patch_count = 1;
    EXPECT_NO_THROW(detail::CheckResult(f.declaration, f.geometry, result));
    EXPECT_EQ(detail::SelectedDeclarationRow(f.geometry, result, 0), SIZE_MAX);
    EXPECT_EQ(detail::SelectedDeclarationRow(f.geometry, result, result.rows.size()-1), f.packing.master_rows[1]);
    result.rows.back().choice.ordered_master = 3;
    EXPECT_THROW(detail::CheckResult(f.declaration, f.geometry, result), std::exception);
    result.rows.back().choice.ordered_master = 2;
    EXPECT_NO_THROW(detail::CheckResult(f.declaration, f.geometry, result));
}
}
