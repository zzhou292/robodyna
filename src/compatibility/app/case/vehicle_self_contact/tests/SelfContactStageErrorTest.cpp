#include "../SelfContactStageError.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <initializer_list>

namespace crash::cases::vehicle_self_contact {
namespace {

using Count = tlfea::contact::SelfContactTransactionCountKind;
using Report = tlfea::contact::SelfContactTransactionReport;
using Status = tlfea::contact::SelfContactTransactionStatus;
using Stage = SelfContactRuntimeStage;

Report ResourceReport(Count kind, std::size_t value) {
    Report report;
    report.status = Status::ResourceLimit;
    report.count_kind = kind;
    report.candidate = value;
    report.message = "bounded transaction resource exhausted";
    return report;
}

TEST(SelfContactStageError, ExactAcceptedCountIsRequiredForceCapacity) {
    const auto report = ResourceReport(Count::ExactAcceptedEvents, 65);
    const SelfContactStageError error(report, Stage::AcceptedAssembly, 64);
    EXPECT_EQ(error.required_events(), 65u);
    EXPECT_EQ(error.required_events_lower_bound(), 0u);
    EXPECT_EQ(error.report().count_kind, Count::ExactAcceptedEvents);
    EXPECT_EQ(error.report().candidate, 65u);
    EXPECT_EQ(error.stage(), Stage::AcceptedAssembly);
    EXPECT_STREQ(error.what(),
                 "accepted self-contact assembly: bounded transaction resource exhausted");
}

TEST(SelfContactStageError, CensusLowerBoundNeverBecomesExactRequirement) {
    // A compact identity census can exhaust below, at, or above the force cap.
    // All are useful lower bounds; none establish the complete requirement.
    for (const std::size_t count : {13u, 64u, 65u, 4096u}) {
        SCOPED_TRACE(count);
        const auto report = ResourceReport(Count::AcceptedEventsLowerBound, count);
        const SelfContactStageError error(report, Stage::AcceptedAssembly, 64);
        EXPECT_EQ(error.required_events(), 0u);
        EXPECT_EQ(error.required_events_lower_bound(), count);
        EXPECT_EQ(error.report().count_kind, Count::AcceptedEventsLowerBound);
        EXPECT_EQ(error.report().candidate, count);
    }
}

TEST(SelfContactStageError, CandidateOrdinalNeverBecomesEventCount) {
    // Broadphase/discovery diagnostics can use candidate as an ordinal even
    // when ResourceLimit is returned. Text must not upgrade its type.
    auto report = ResourceReport(Count::None, 1000000);
    report.pair = 812;
    report.message = "required events 1000000";
    const SelfContactStageError error(report, Stage::AcceptedAssembly, 64);
    EXPECT_EQ(error.required_events(), 0u);
    EXPECT_EQ(error.required_events_lower_bound(), 0u);
    EXPECT_EQ(error.report().candidate, 1000000u);
    EXPECT_EQ(error.report().pair, 812u);
    EXPECT_EQ(error.report().count_kind, Count::None);
}

TEST(SelfContactStageError, CandidateSealAndOtherStatusesDoNotAdvertiseEventCounts) {
    for (const auto kind : {Count::ExactAcceptedEvents,
                            Count::AcceptedEventsLowerBound}) {
        const auto report = ResourceReport(kind, 1000);
        const SelfContactStageError seal(report, Stage::CandidateSeal, 64);
        EXPECT_EQ(seal.required_events(), 0u);
        EXPECT_EQ(seal.required_events_lower_bound(), 0u);
        EXPECT_EQ(seal.report().count_kind, kind);
        EXPECT_EQ(seal.report().candidate, 1000u);
        EXPECT_EQ(seal.stage(), Stage::CandidateSeal);
        EXPECT_STREQ(seal.what(),
                     "candidate self-contact seal: bounded transaction resource exhausted");

        for (const auto status : {Status::Ok, Status::InvalidInput,
                                  Status::CrossingFailure}) {
            auto other_report = report;
            other_report.status = status;
            const SelfContactStageError other(
                other_report, Stage::AcceptedAssembly, 64);
            EXPECT_EQ(other.required_events(), 0u);
            EXPECT_EQ(other.required_events_lower_bound(), 0u);
            EXPECT_EQ(other.report().status, status);
        }
    }
}

TEST(SelfContactStageError, SentinelAndNonExceedingExactCountsStayUnavailable) {
    for (const auto kind : {Count::None, Count::ExactAcceptedEvents,
                            Count::AcceptedEventsLowerBound}) {
        const SelfContactStageError sentinel(
            ResourceReport(kind, SIZE_MAX), Stage::AcceptedAssembly, 0);
        EXPECT_EQ(sentinel.required_events(), 0u);
        EXPECT_EQ(sentinel.required_events_lower_bound(), 0u);
    }
    for (const std::size_t count : {0u, 63u, 64u}) {
        const SelfContactStageError fits(
            ResourceReport(Count::ExactAcceptedEvents, count),
            Stage::AcceptedAssembly, 64);
        EXPECT_EQ(fits.required_events(), 0u);
        EXPECT_EQ(fits.required_events_lower_bound(), 0u);
        EXPECT_EQ(fits.report().candidate, count);
    }
}

TEST(SelfContactStageError, TypedDiagnosticSurvivesReportCopy) {
    auto report = ResourceReport(Count::None, 33);
    report.status = Status::CrossingFailure;
    report.pair = 7;
    report.discovery_task = 11;
    report.nonlinear_subdivision_work = 127;
    report.nonlinear_subdivision_depth = 6;
    report.nonlinear_subdivision_work_exhausted = true;
    report.offending_half_thickness_m[0] = .0005;
    report.offending_half_thickness_m[1] = .001;
    const SelfContactStageError error(report, Stage::CandidateSeal, 64);
    report = {};

    EXPECT_EQ(error.report().status, Status::CrossingFailure);
    EXPECT_EQ(error.report().candidate, 33u);
    EXPECT_EQ(error.report().pair, 7u);
    EXPECT_EQ(error.report().discovery_task, 11u);
    EXPECT_EQ(error.report().nonlinear_subdivision_work, 127u);
    EXPECT_EQ(error.report().nonlinear_subdivision_depth, 6u);
    EXPECT_TRUE(error.report().nonlinear_subdivision_work_exhausted);
    EXPECT_DOUBLE_EQ(error.report().offending_half_thickness_m[0], .0005);
    EXPECT_DOUBLE_EQ(error.report().offending_half_thickness_m[1], .001);
    EXPECT_EQ(error.required_events(), 0u);
    EXPECT_EQ(error.required_events_lower_bound(), 0u);
}

}  // namespace
}  // namespace crash::cases::vehicle_self_contact
