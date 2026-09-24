#include "Fixture.h"
#include "../Observe.h"
#include "../Document.h"
#include "case/vehicle_run/SelfContactDocument.h"
#include "case/vehicle_run/SelfContactSummary.h"
#include <gtest/gtest.h>
#include <functional>
#include <limits>
#include <sstream>
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::cases::vehicle_run::contact_diagnostics::test {
TEST(VehicleContactDiagnostics, DisabledValuesPreserveExistingSummaryAndProgressShape) {
    const auto disabled=Copy({});
    EXPECT_FALSE(disabled.enabled);
    SelfContactTotals totals;totals.available=true;
    totals.performance=disabled;
    EXPECT_FALSE(vehicle_run::detail::SelfContactDocument(totals).HasMember("performance_diagnostics"));
    std::ostringstream progress;WriteProgress(progress,disabled);EXPECT_TRUE(progress.str().empty());
}
TEST(VehicleContactDiagnostics, ActualNativeSliceCountsRemainDistinctAndBindCommittedPhase) {
    const auto source=Input();
    const auto snapshot=Committed(source,7,2,9);
    ASSERT_TRUE(snapshot.enabled);ASSERT_TRUE(snapshot.phase_matches);
    const auto document=Document(snapshot);
    ASSERT_TRUE(document["matches_committed_interval"].GetBool());
    const auto& candidate=document["candidate"];
    EXPECT_EQ(candidate["native_submitted_pairs"].GetUint64(),37u);
    EXPECT_EQ(candidate["native_batches"].GetUint64(),2u);
    EXPECT_EQ(candidate["native_work"].GetUint64(),90u);
    EXPECT_EQ(candidate["discovery"]["raw_feature_candidates"].GetUint64(),251u);
    EXPECT_EQ(candidate["discovery"]["feature_candidates"].GetUint64(),177u);
    EXPECT_EQ(candidate["stages"][6]["host_wall_ns"].GetUint64(),1700u);
    EXPECT_FALSE(candidate["stages"][0]["timing_available"].GetBool());
    EXPECT_FALSE(candidate["stages"][0].HasMember("host_wall_ns"));
    SelfContactTotals totals;totals.available=true;totals.last_exact_crossing_pairs=999;
    totals.performance=snapshot;
    const auto summary=vehicle_run::detail::SelfContactDocument(totals);
    EXPECT_EQ(summary["last_exact_crossing_pairs"].GetUint64(),999u);
    EXPECT_EQ(summary["performance_diagnostics"]["candidate"]["native_submitted_pairs"].GetUint64(),37u);
}
TEST(VehicleContactDiagnostics, ForeignIncompleteAndFailedPhasesAreUnavailableWithoutThrowing) {
    using Native=tlfea::contact::SelfContactTransactionDiagnostics;
    const std::function<void(Native&)> mutations[]{
        [](auto& x){++x.accepted.owner_id;},[](auto& x){++x.candidate.base_epoch;},
        [](auto& x){++x.candidate.attempt;},[](auto& x){x.candidate.finished=false;},
        [](auto& x){x.accepted.authenticated=false;},[](auto& x){x.candidate.succeeded=false;},
        [](auto& x){x.accepted.enabled=false;},[](auto& x){x.candidate.entered=false;}};
    for(const auto& mutate:mutations) {
        auto source=Input();mutate(source);
        const auto snapshot=Committed(source,7,2,9);
        EXPECT_TRUE(snapshot.enabled);EXPECT_FALSE(snapshot.phase_matches);
        const auto document=Document(snapshot);
        EXPECT_FALSE(document["matches_committed_interval"].GetBool());
        EXPECT_FALSE(document.HasMember("candidate"));
        std::ostringstream out;WriteProgress(out,snapshot);
        EXPECT_NE(out.str().find("contact_diagnostics_available=0"),std::string::npos);
        EXPECT_EQ(out.str().find("native_submitted_pairs"),std::string::npos);
    }
}
TEST(VehicleContactDiagnostics, ClockFailureBackwardSampleAndSaturationNeverBecomeZeroTimings) {
    for(unsigned failure=0;failure<3;++failure) {
        auto source=Input();
        if(failure==0){source.candidate.clock_failures=1;source.candidate.stages[6].valid_samples=1;}
        if(failure==1){source.candidate.backward_samples=1;source.candidate.stages[6].valid_samples=1;}
        if(failure==2){source.candidate.counter_saturated=true;source.candidate.counts_complete=false;}
        const auto snapshot=Committed(source,7,2,9);
        EXPECT_TRUE(snapshot.phase_matches);
        const auto document=Document(snapshot);
        const auto& stage=document["candidate"]["stages"][6];
        EXPECT_FALSE(stage["timing_available"].GetBool());
        EXPECT_FALSE(stage.HasMember("host_wall_ns"));
        std::ostringstream out;WriteProgress(out,snapshot);
        EXPECT_NE(out.str().find("candidate_native_crossing_timing_available=0"),std::string::npos);
        EXPECT_EQ(out.str().find("candidate_native_crossing_host_s="),std::string::npos);
    }
}
TEST(VehicleContactDiagnostics, FailedLastAttemptRemainsSeparateFromCommittedObservations) {
    auto source=Input();source.candidate.succeeded=false;source.candidate.counts_complete=false;
    source.candidate.stages[6].failures=1;source.candidate.discovery.failures=1;
    const auto raw=Copy(source);
    EXPECT_FALSE(raw.committed_scope);
    const auto document=Document(raw);
    EXPECT_FALSE(document["candidate"]["succeeded"].GetBool());
    EXPECT_FALSE(document["candidate"]["counts_complete"].GetBool());
    EXPECT_EQ(document["candidate"]["native_submitted_pairs"].GetUint64(),37u);
    EXPECT_FALSE(Committed(source,7,2,9).phase_matches);
}
TEST(VehicleContactDiagnostics, FixedWorstCaseScalarDocumentFitsExistingSummaryAllowance) {
    auto source=Input();
    for(auto* phase:{&source.accepted,&source.candidate}) {
        phase->owner_id=phase->base_epoch=phase->attempt=UINT64_MAX;
        phase->native_batches=phase->native_submitted_pairs=phase->native_work=UINT64_MAX;
        phase->clock_failures=phase->backward_samples=UINT64_MAX;
        phase->discovery.calls=UINT64_MAX;
        phase->discovery.failures=UINT64_MAX;
        phase->discovery.triangle_references=UINT64_MAX;
        phase->discovery.triangles=UINT64_MAX;
        phase->discovery.vertex_references=UINT64_MAX;
        phase->discovery.vertices=UINT64_MAX;
        phase->discovery.edge_references=UINT64_MAX;
        phase->discovery.edges=UINT64_MAX;
        phase->discovery.raw_feature_candidates=UINT64_MAX;
        phase->discovery.feature_candidates=UINT64_MAX;
        phase->discovery.raw_intersections=UINT64_MAX;
        phase->discovery.intersections=UINT64_MAX;
        phase->discovery.potential_tasks=UINT64_MAX;
        phase->discovery.local_masked_tasks=UINT64_MAX;
        phase->discovery.exact_executed_tasks=UINT64_MAX;
        for(auto& counter:phase->stages)counter={UINT64_MAX,UINT64_MAX,UINT64_MAX,UINT64_MAX,UINT64_MAX};
    }
    const auto document=Document(Copy(source));
    rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
    EXPECT_LT(buffer.GetSize(),16u<<10);
    EXPECT_LE(sizeof(Snapshot),2048u);
    EXPECT_LE(sizeof(SelfContactTotals),4096u);
}
} // namespace crash::cases::vehicle_run::contact_diagnostics::test
