#include "Fixture.h"
#include <cmath>
#include <type_traits>
namespace crash::cases::vehicle_startup::finalization_test {
TEST(TiedFinalizationSourceValues, OrdinarySingleInterfaceReceiptBindsOriginalCardsAndNativePhase) {
    Fixture f;
    const auto receipt = f.Receipt();
    EXPECT_EQ(receipt.type2_count,1u);
    EXPECT_EQ(receipt.type2_ordinal,1u);
    EXPECT_EQ(receipt.is1,2);
    EXPECT_EQ(receipt.level,28);
    EXPECT_EQ(receipt.ignore,2);
    EXPECT_EQ(receipt.projection,1);
    EXPECT_EQ(receipt.search_distance,0);
    EXPECT_EQ(receipt.prior_connection_count,0u);
    EXPECT_EQ(receipt.multiple_connection_count,0u);
    EXPECT_EQ(receipt.unique_original_slaves,f.declaration.slave_nodes.size());
    EXPECT_EQ(receipt.contact_source,f.declaration.contact_source);
    EXPECT_EQ(receipt.phase,TiedFinalizationPhase::FreshPhysicalCoefficientsBeforeIniend);
    static_assert(!std::is_copy_assignable_v<TiedSearchFinalized>);
}
TEST(TiedFinalizationSourceValues, AdditionalInterfaceAndOmittedCensusOrSourceIdentityReject) {
    Fixture f;
    const auto original = f.canonical.canonical_bytes;
    f.AlterCanonical([](auto& doc) {
        auto& file = doc["source_files"]["auxiliary.key"];
        auto& row = file["blocks"][0];
        row["keyword"].SetString("*CONTACT_TIED_NODES_TO_SURFACE",doc.GetAllocator());
        file["keyword_counts"].AddMember("*CONTACT_TIED_NODES_TO_SURFACE",1,doc.GetAllocator());
    });
    EXPECT_THROW(f.Receipt(),std::exception);
    f.canonical.canonical_bytes = original;
    f.AlterCanonical([](auto& doc) {
        doc["source_files"]["yaris-coarse-v1l.key"]["keyword_counts"]
            ["*CONTACT_TIED_SHELL_EDGE_TO_SURFACE"].SetUint(2);
    });
    EXPECT_THROW(f.Receipt(),std::exception);
    f.canonical.canonical_bytes = original;
    f.AlterCanonical([](auto& doc) {
        for (auto& row : doc["source_files"]["yaris-coarse-v1l.key"]["blocks"].GetArray())
            if (std::string(row["keyword"].GetString()) == "*CONTACT_TIED_SHELL_EDGE_TO_SURFACE")
                row["source_block_sha256"].SetString(std::string(64,'b').c_str(),doc.GetAllocator());
    });
    EXPECT_THROW(f.Receipt(),std::exception);
    f.canonical.canonical_bytes = original;
    EXPECT_EQ(f.Receipt().type2_count,1u);
}
TEST(TiedFinalizationSourceValues, ChangedSourceSetTypeAndDuplicateNSVRejectWithoutGuessingProfile) {
    Fixture f;
    const auto contact = f.declaration.sources[f.declaration.contact_source];
    f.declaration.sources[f.declaration.contact_source].cards[0].second.replace(20,10,"        -1");
    EXPECT_THROW(f.Receipt(),std::exception);
    f.declaration.sources[f.declaration.contact_source] = contact;
    const auto last = f.declaration.slave_nodes.back().id;
    f.declaration.slave_nodes.back().id = f.declaration.slave_nodes.front().id;
    EXPECT_THROW(f.Receipt(),std::exception);
    f.declaration.slave_nodes.back().id = last;
    EXPECT_EQ(f.Receipt().is1,2);
}
TEST(TiedFinalizationSourceValues, ExactSourceRowsRepeatedTriangleAndNativeDispositionRemainSeparateFromPatchReadiness) {
    Fixture f;
    f.result.rows.front().choice.projection.s = 1.6;
    auto& unmatched = f.result.rows[1];
    unmatched.choice.matched = false;
    unmatched.choice.ordered_master = 0;
    --f.result.matched_count;
    --f.result.singular_patch_count;
    const auto inputs = f.Inputs();
    native_search::FinalizedSearch finalized;
    ASSERT_TRUE(native_search::FinalizeSearch(inputs.View(f.geometry),&finalized));
    EXPECT_EQ(finalized.data()->dispositions[0],native_search::FinalizationDisposition::OutsideParameters);
    EXPECT_EQ(finalized.data()->dispositions[1],native_search::FinalizationDisposition::Unmatched);
    EXPECT_EQ(finalized.data()->slaves.size()+2,f.result.rows.size());
    for (std::size_t m = 0; m < inputs.masters.size(); ++m)
        EXPECT_EQ(inputs.masters[m],f.geometry.masters[m].working_nodes);
    for (std::size_t m = 0; m < inputs.main_nodes.size(); ++m)
        EXPECT_EQ(f.geometry.canonical_nodes[inputs.main_nodes[m]],f.declaration.master_nodes[m]);
    for (auto row : finalized.data()->slaves) {
        EXPECT_EQ(f.result.rows[row].force_patch_status,native_search::Status::SingularPatch);
        EXPECT_TRUE(std::signbit(f.result.rows[row].choice.projection.s));
    }
}
TEST(TiedFinalizationSourceValues, CompleteBudgetLateAssociationFailureAndExactCapRetry) {
    Fixture f;
    const auto forecast = f.Forecast();
    EXPECT_EQ(forecast.finalizer_reservation_bytes,TiedFinalizationLimits{}.native.max_host_bytes);
    EXPECT_EQ(forecast.total_host_bytes,forecast.retained_source_bytes+forecast.retained_assessment_bytes+
        forecast.input_staging_bytes+forecast.metadata_reservation_bytes+forecast.finalizer_reservation_bytes+forecast.fixed_bytes);
    TiedFinalizationLimits cap;
    cap.host_bytes = forecast.total_host_bytes-1;
    EXPECT_THROW(f.Forecast(cap),std::exception);
    ++cap.host_bytes;
    EXPECT_EQ(f.Forecast(cap).total_host_bytes,forecast.total_host_bytes);
    const auto packed = f.Inputs();
    native_search::FinalizedSearch output;
    ASSERT_TRUE(native_search::FinalizeSearch(packed.View(f.geometry),&output));
    const auto* prior = output.data();
    const auto saved = f.geometry.secondary_working_nodes.back();
    f.geometry.secondary_working_nodes.back() = UINT32_MAX;
    EXPECT_THROW(f.Inputs(),std::exception);
    EXPECT_EQ(output.data(),prior);
    f.geometry.secondary_working_nodes.back() = saved;
    auto retry = f.Inputs();
    EXPECT_EQ(retry.main_nodes,packed.main_nodes);
    EXPECT_EQ(retry.masters,packed.masters);
    ASSERT_TRUE(native_search::FinalizeSearch(retry.View(f.geometry),&output));
    EXPECT_EQ(output.data()->slaves.size(),f.result.rows.size());
    cap.native.max_slaves = f.result.rows.size()-1;
    EXPECT_THROW(f.Forecast(cap),std::exception);
    cap.native.max_slaves++;
    EXPECT_NO_THROW(f.Forecast(cap));
}
}
