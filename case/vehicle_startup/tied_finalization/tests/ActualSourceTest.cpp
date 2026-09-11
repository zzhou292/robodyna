#include "../Internal.h"
#include "../../tied_search/tests/ActualGeometry.h"
#include "lib_utest/qualification/tied_shell_finalization/NativeOracle.h"
#include <gtest/gtest.h>
#include <iostream>
namespace crash::cases::vehicle_startup::finalization_test {
TEST(TiedFinalizationSourceActual, CompleteOriginalAssessmentMatchesNativeFinalizationWithRetainedSourceLifetime) {
    const auto finalized = [] {
        const auto assessment = TiedSearchAssessment::Prepare(test::OriginalTiedGeometry());
        const auto forecast = TiedSearchFinalized::Forecast(assessment);
        std::cout << "Tied finalization forecast: source=" << forecast.retained_source_bytes
                  << " assessment=" << forecast.retained_assessment_bytes
                  << " inputs=" << forecast.input_staging_bytes
                  << " metadata=" << forecast.metadata_reservation_bytes
                  << " native_reservation=" << forecast.finalizer_reservation_bytes
                  << " total=" << forecast.total_host_bytes << " B\n" << std::flush;
        return TiedSearchFinalized::Prepare(assessment);
    }(); // Source, geometry and assessment construction temporaries are gone.
    const auto& assessment = finalized.assessment();
    const auto& geometry = assessment.geometry();
    const auto& declaration = geometry.packing().declaration().data();
    const auto& data = finalized.data();
    ASSERT_EQ(data.dispositions.size(),11165u);
    ASSERT_EQ(data.slaves.size(),11165u);
    ASSERT_EQ(data.main_inverse.size(),183457u);
    ASSERT_EQ(geometry.data().masters.size(),171813u);
    EXPECT_EQ(finalized.classification(),TiedAssessmentReadiness::Pending);
    EXPECT_EQ(finalized.receipt().source_file_count,4u);
    EXPECT_EQ(finalized.receipt().source_contact_count,3u);
    EXPECT_EQ(finalized.receipt().type2_count,1u);
    EXPECT_EQ(finalized.receipt().type2_ordinal,1u);
    EXPECT_EQ(finalized.receipt().unique_original_slaves,11165u);
    EXPECT_EQ(finalized.receipt().is1,2);
    EXPECT_EQ(finalized.receipt().multiple_connection_count,0u);
    const auto inputs = tied_finalization_detail::Pack(declaration,geometry.packing().data(),
        geometry.data(),assessment.result());
    const auto native = tied_finalization_test::Native(inputs.View(geometry.data()));
    tied_finalization_test::Compare(inputs.View(geometry.data()),data,native);
    ASSERT_FALSE(HasFailure());
    for (std::size_t compact = 0; compact < data.slaves.size(); ++compact) {
        const auto original = data.slaves[compact];
        EXPECT_EQ(data.dispositions[original],native_search::FinalizationDisposition::Kept);
        EXPECT_EQ(finalized.secondary(compact).id,declaration.slave_nodes[original].id);
        EXPECT_EQ(finalized.secondary(compact).canonical_index,declaration.slave_nodes[original].canonical_index);
        const auto rank = data.selected_masters[compact];
        const auto source_row = geometry.data().masters.at(rank-1).declaration_row;
        EXPECT_EQ(finalized.selected_master(compact).id,declaration.masters[source_row].id);
        EXPECT_EQ(assessment.result().rows[original].force_patch_status,native_search::Status::Success);
    }
    ASSERT_EQ(data.messages.size(),7u);
    for (const auto& message : data.messages)
        EXPECT_EQ(message.action,native_search::NativeMessageAction::Flush);
    EXPECT_LE(data.startup_payload_bytes,finalized.forecast().finalizer_reservation_bytes);
    TiedFinalizationLimits limits;
    limits.host_bytes = finalized.forecast().total_host_bytes-1;
    EXPECT_THROW(TiedSearchFinalized::Prepare(assessment,limits),std::exception);
    EXPECT_EQ(finalized.data().slaves.size(),11165u);
    ++limits.host_bytes;
    const auto retry = TiedSearchFinalized::Prepare(assessment,limits);
    EXPECT_EQ(retry.data().slaves,data.slaves);
    EXPECT_EQ(retry.data().main_nodes,data.main_nodes);
    EXPECT_EQ(retry.data().selected_masters,data.selected_masters);
    EXPECT_EQ(retry.data().st,data.st);
    std::cout << "Tied finalization: kept=" << data.slaves.size()
              << " used_master_nodes=" << data.main_nodes.size()
              << " original_master_nodes=" << data.main_inverse.size()
              << " native_message_calls=" << data.messages.size()
              << " owned_payload=" << data.owned_payload_bytes << " B\n";
    RecordProperty("kept_source_slaves",std::to_string(data.slaves.size()));
    RecordProperty("compacted_master_nodes",std::to_string(data.main_nodes.size()));
    RecordProperty("total_host_forecast_bytes",std::to_string(finalized.forecast().total_host_bytes));
}
}
