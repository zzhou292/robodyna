#include "../FailureRun.h"
#include "../Publication.h"
#include "case/vehicle_run/SelfContactDocument.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <stdexcept>

namespace crash::cases::vehicle_run::diagnostics {
namespace {
namespace fs=std::filesystem;
using Directory=output::full_shell::test::Directory;
using Stage=vehicle_self_contact::SelfContactRuntimeStage;
using Error=vehicle_self_contact::SelfContactStageError;
using Status=tlfea::contact::SelfContactTransactionStatus;
Result Rejected() {
    Result value;
    value.loop.kind=StopKind::PhysicsRejected;
    value.loop.progress.accepted={17,.0000034};
    tlfea::contact::SelfContactTransactionReport report;
    report.status=Status::UnresolvedCandidate;
    report.message="original native rejection";
    report.pair=29;
    report.crossing_reason=tlfea::contact::RepresentedIntervalReason::WorkExhausted;
    value.rejected_self_contact.emplace(report,Stage::CandidateSeal,65536);
    return value;
}
TEST(VehicleFailureDestination, CreatesNothingAndRejectsRunOrSourceOverlap) {
    Directory directory;
    const auto run=directory.path/"run";
    const auto source=directory.path/"source";
    fs::create_directory(run);fs::create_directory(source);
    const auto destination=directory.path/"failure";
    EXPECT_EQ(CheckDestination(destination,run,{source}),destination);
    EXPECT_FALSE(fs::exists(destination));
    EXPECT_TRUE(fs::is_empty(run));
    EXPECT_THROW(CheckDestination(run/"failure",run,{source}),std::exception);
    EXPECT_THROW(CheckDestination(source/"failure",run,{source}),std::exception);
    EXPECT_THROW(CheckDestination(directory.path,run),std::exception);
    EXPECT_THROW(CheckDestination(directory.path/"missing"/"failure",run),std::exception);
    EXPECT_THROW(CheckDestination({},run),std::exception);
    EXPECT_THROW(CheckDestination(destination,{}),std::exception);
}
TEST(VehicleFailureDestination, RejectsSymlinkAliasesAndExistingCompanions) {
    Directory directory;
    const auto run=directory.path/"run";
    fs::create_directory(run);
    fs::create_directory_symlink(run,directory.path/"alias");
    EXPECT_THROW(CheckDestination(directory.path/"alias"/"failure",run),std::exception);
    fs::create_symlink(directory.path/"absent",directory.path/"dangling");
    EXPECT_THROW(CheckDestination(directory.path/"dangling",run),std::exception);
    fs::create_directory(directory.path/"failure");
    EXPECT_THROW(CheckDestination(directory.path/"failure",run),std::exception);
}
TEST(VehicleFailureBudget, IncludesExactCompanionsAndFailsOneByteShortWithoutDeviceChange) {
    Forecast forecast;
    forecast.contact.self_contact.emplace();
    forecast.complete_host_bytes=1701;forecast.complete_archive_bytes=902;
    forecast.caps.host_bytes=1701+(8u<<20)+1024;
    forecast.caps.archive_bytes=902+(2u<<20);
    forecast.contact.device_bytes=765;
    const auto admitted=Preflight(forecast);
    EXPECT_EQ(admitted.host_bytes,forecast.caps.host_bytes);
    EXPECT_EQ(admitted.archive_bytes,forecast.caps.archive_bytes);
    EXPECT_EQ(forecast.contact.device_bytes,765u);
    --forecast.caps.host_bytes;
    EXPECT_THROW(Preflight(forecast),std::exception);
    ++forecast.caps.host_bytes;--forecast.caps.archive_bytes;
    EXPECT_THROW(Preflight(forecast),std::exception);
    ++forecast.caps.archive_bytes;forecast.contact.self_contact.reset();
    EXPECT_THROW(Preflight(forecast),std::exception);
}
TEST(VehicleFailurePublication, NoRejectionMissingPairAndIncompleteCaptureNeverExport) {
    auto run=Rejected();
    const auto before=vehicle_run::detail::SelfContactErrorDocument(*run.rejected_self_contact);
    unsigned calls=0;
    const auto exporter=[&](const fs::path&) {++calls;return std::string(64,'a');};
    detail::CaptureEvidence evidence;
    EXPECT_EQ(detail::PublishEvidence(run,evidence,"absent",exporter).status,
        FailureStatus::RejectionWithoutPair);
    run.loop.kind=StopKind::Completed;
    EXPECT_EQ(detail::PublishEvidence(run,evidence,"absent",exporter).status,FailureStatus::NoRejection);
    run.loop.kind=StopKind::PhysicsRejected;
    evidence.seen=true;evidence.error="capture limit exhausted";
    const auto incomplete=detail::PublishEvidence(run,evidence,"absent",exporter);
    EXPECT_EQ(incomplete.status,FailureStatus::CaptureIncomplete);
    EXPECT_EQ(incomplete.error,evidence.error);
    EXPECT_EQ(calls,0u);
    EXPECT_TRUE(before==vehicle_run::detail::SelfContactErrorDocument(*run.rejected_self_contact));
}
TEST(VehicleFailurePublication, ArbitraryLaterEpochMatchesOriginalTypedFailure) {
    auto run=Rejected();
    const detail::CaptureEvidence evidence{true,true,true,17,run.rejected_self_contact->report(),{}};
    unsigned calls=0;
    const auto exporter=[&](const fs::path& path) {
        ++calls;EXPECT_EQ(path,"requested-failure");return std::string(64,'a');
    };
    const auto result=detail::PublishEvidence(run,evidence,"requested-failure",exporter);
    EXPECT_EQ(result.status,FailureStatus::Captured);
    EXPECT_EQ(result.manifest,fs::path("requested-failure")/"failure.json");
    EXPECT_EQ(result.sha256,std::string(64,'a'));EXPECT_EQ(calls,1u);
    EXPECT_EQ(run.loop.kind,StopKind::PhysicsRejected);
    EXPECT_EQ(run.loop.progress.accepted.epoch,17u);
    EXPECT_EQ(run.rejected_self_contact->report().pair,29u);
    EXPECT_THROW(detail::CheckCapturedFailure(run,evidence.report,16),std::exception);
    auto changed=evidence.report;++changed.pair;
    EXPECT_THROW(detail::CheckCapturedFailure(run,changed,17),std::exception);
    run.rejected_self_contact.emplace(evidence.report,Stage::AcceptedAssembly,65536);
    EXPECT_THROW(detail::CheckCapturedFailure(run,evidence.report,17),std::exception);
}
TEST(VehicleFailurePublication, ExportAndConsistencyErrorsPreserveNativeRejectionAndEvidence) {
    auto run=Rejected();
    const auto before=vehicle_run::detail::SelfContactErrorDocument(*run.rejected_self_contact);
    detail::CaptureEvidence evidence{true,true,true,17,run.rejected_self_contact->report(),{}};
    const auto failed=detail::PublishEvidence(run,evidence,"requested",[](const fs::path&)->std::string {
        throw std::runtime_error("injected export I/O failure");
    });
    EXPECT_EQ(failed.status,FailureStatus::ExportFailed);
    EXPECT_EQ(failed.error,"injected export I/O failure");
    --evidence.accepted_epoch;
    const auto inconsistent=detail::PublishEvidence(run,evidence,"requested",[](const fs::path&) {
        return std::string(64,'b');
    });
    EXPECT_EQ(inconsistent.status,FailureStatus::CaptureIncomplete);
    EXPECT_FALSE(inconsistent.error.empty());
    EXPECT_EQ(inconsistent.manifest,fs::path("requested")/"failure.json");
    EXPECT_EQ(inconsistent.sha256,std::string(64,'b'));
    EXPECT_TRUE(before==vehicle_run::detail::SelfContactErrorDocument(*run.rejected_self_contact));
    EXPECT_EQ(run.loop.progress.accepted.epoch,17u);
}
} // namespace
} // namespace crash::cases::vehicle_run::diagnostics
