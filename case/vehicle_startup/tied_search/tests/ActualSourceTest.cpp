#include "../../TiedSearchAssessment.h"
#include "output/full_shell/static_bundle/tests/ActualMappingSupport.h"
#include "modelio/tied_shell/search_geometry/tests/BucketOracle.h"
#include "lib_utest/qualification/tied_shell_search/NativeOracle.h"
#include <gtest/gtest.h>
#include <iostream>
namespace crash::cases::vehicle_startup::test {
namespace {
tied::TiedShellSearchGeometry OriginalGeometry() {
    auto inputs = tied::source::test::ActualInputs();
    inputs.scope_report.bytes = 13212691;
    inputs.scope_report.sha256 = "fdb51869dfd3f4de265bb4494a2d0f904c5c466bf9962b71ce19e9098a761ff0";
    const auto canonical = tied::source::CanonicalSource::Read(inputs);
    const auto member = output::ReadBounded(std::getenv("ROBO_STATIC_MEMBER"), 42846753);
    const auto declaration = tied::TiedShellDeclaration::Prepare(canonical, member);
    const auto packing = tied::TiedShellPacking::Prepare(declaration);
    return tied::TiedShellSearchGeometry::Prepare(packing, member);
}
}
TEST(TiedAssessmentActual, Complete11165SourceRowsKeepOriginalIdentityAndPendingNativeStages) {
    const auto assessment = [] {
        const auto geometry = OriginalGeometry();
        const auto forecast = TiedSearchAssessment::Forecast(geometry);
        std::cout << "Tied assessment preflight: source=" << forecast.source_payload_bytes
                  << " staging=" << forecast.input_staging_bytes << " fixed=" << forecast.fixed_bytes
                  << " driver_reservation=" << forecast.driver_host_reservation_bytes
                  << " total_host=" << forecast.total_host_bytes << " B\n" << std::flush;
        return TiedSearchAssessment::Prepare(geometry);
    }(); // Every original temporary handle and member buffer has gone away.
    ASSERT_EQ(assessment.result().rows.size(), 11165u);
    ASSERT_EQ(assessment.geometry().data().masters.size(), 171813u);
    EXPECT_EQ(assessment.finalization(), TiedAssessmentReadiness::Pending);
    EXPECT_EQ(assessment.classification(), TiedAssessmentReadiness::Pending);
    std::size_t matched = 0, warning = 0, blocked = 0;
    const auto& d = assessment.geometry().packing().declaration().data();
    for (std::size_t s = 0; s < 11165; ++s) {
        const auto& node = assessment.secondary(s);
        ASSERT_EQ(node.id, d.slave_nodes[s].id);
        ASSERT_EQ(node.canonical_index, d.slave_nodes[s].canonical_index);
        const auto* master = assessment.selected_master(s);
        const auto& row = assessment.result().rows[s];
        ASSERT_EQ(master != nullptr, row.choice.matched);
        if (!master) {
            EXPECT_EQ(assessment.selected_part_id(s), 0u);
            continue;
        }
        ++matched;
        warning += row.choice.projection.outside_warning;
        blocked += row.force_patch_status != native_search::Status::Success;
        ASSERT_LE(row.choice.ordered_master, 171813u);
        const auto declared = assessment.geometry().data().masters[row.choice.ordered_master-1].declaration_row;
        ASSERT_EQ(master->id, d.masters[declared].id);
        ASSERT_EQ(assessment.selected_part_id(s), d.parts[master->part_index].id);
        EXPECT_FALSE(assessment.geometry().PhysicalOwnNode(s, row.choice.ordered_master-1));
    }
    EXPECT_EQ(matched, assessment.result().matched_count);
    const auto& b = assessment.result().budget;
    EXPECT_LE(b.startup_host_bytes, assessment.forecast().driver_host_reservation_bytes);
    EXPECT_LE(b.device_bytes, assessment.forecast().device_limit_bytes);
    std::cout << "Tied assessment: pairs=" << assessment.result().pair_count << " matched=" << matched
              << " unmatched=" << 11165-matched << " projection_outside_warning=" << warning
              << " selected_patch_blocked=" << blocked << " host_driver=" << b.startup_host_bytes
              << " device=" << b.device_bytes << " sort_scratch=" << b.sort_scratch_bytes
              << " scan_scratch=" << b.scan_scratch_bytes << " pair_capacity=" << b.pair_capacity << '\n';
    // Independent native bucket enumeration consumes source geometry, never
    // production boxes or candidate pairs. Compare its complete NSV coverage.
    const auto native = tied::test::NativeBucket(assessment.geometry());
    ASSERT_EQ(native.selected.size(), assessment.result().rows.size());
    std::vector<std::size_t> pair_counts(native.selected.size(), 0);
    for (const auto& pair : native.pairs) {
        ASSERT_GE(pair[1], 1);
        ASSERT_LE(std::size_t(pair[1]), pair_counts.size());
        ++pair_counts[pair[1]-1];
    }
    const auto& geometry = assessment.geometry().data();
    for (std::size_t s = 0; s < native.selected.size(); ++s) {
        SCOPED_TRACE(s);
        const auto& row = assessment.result().rows[s];
        ASSERT_EQ(row.choice.matched, native.selected[s] != 0);
        ASSERT_EQ(row.within_native_bounds, pair_counts[s]);
        EXPECT_EQ(row.excluded_own_node, 0u); // Qualified source populations are disjoint.
        if (!row.choice.matched) continue;
        ASSERT_EQ(row.choice.ordered_master, std::uint64_t(native.selected[s]));
        const auto& master = geometry.masters[row.choice.ordered_master-1];
        native_search::WorkingSearchInput packet;
        packet.topology = master.family == tied::SearchShellFamily::Q4 ? native_search::MasterTopology::Quad :
            native_search::MasterTopology::TriangleRepeatedThird;
        packet.master_thickness = master.projection_thickness;
        packet.working_length_to_m = geometry.working_length_to_m;
        for (unsigned local = 0; local < 5; ++local) {
            const auto& x = geometry.working_positions[local == 4 ? geometry.secondary_working_nodes[s] : master.working_nodes[local]];
            const native_search::Vec3 value{x[0],x[1],x[2]};
            if (local == 4) packet.geometry.secondary_position = value;
            else packet.geometry.master_position[local] = value;
        }
        tied_search_test::NativeChoice selected;
        const auto projected = tied_search_test::Native(packet, native.selected[s], selected);
        // Existing owning primitive comparison budget; no new tolerance.
        tied_search_test::Compare(row.choice.projection, projected);
        ASSERT_FALSE(HasFailure());
        EXPECT_DOUBLE_EQ(selected.st[0], native.st[s][0]);
        EXPECT_DOUBLE_EQ(selected.st[1], native.st[s][1]);
        EXPECT_DOUBLE_EQ(selected.distance, native.distance[s]);
    }
    EXPECT_EQ(native.pairs.size(), 31104u);
    EXPECT_EQ(matched, 11165u);
    EXPECT_EQ(warning, 0u);
    EXPECT_EQ(blocked, 0u);
    RecordProperty("native_box_pairs", std::to_string(native.pairs.size()));
    RecordProperty("gpu_candidates", std::to_string(assessment.result().pair_count));
    RecordProperty("matched", std::to_string(matched));
    RecordProperty("host_forecast_bytes", std::to_string(assessment.forecast().total_host_bytes));
    RecordProperty("device_forecast_bytes", std::to_string(b.device_bytes));
    TiedAssessmentLimits tight;
    tight.host_bytes = assessment.forecast().total_host_bytes-1;
    EXPECT_THROW(TiedSearchAssessment::Prepare(assessment.geometry(), tight), std::exception);
    EXPECT_EQ(assessment.result().rows.size(), 11165u);
    tight.host_bytes++;
    EXPECT_EQ(TiedSearchAssessment::Forecast(assessment.geometry(), tight).total_host_bytes,
              assessment.forecast().total_host_bytes);
}
}
