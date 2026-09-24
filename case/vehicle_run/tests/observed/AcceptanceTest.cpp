#include "RunAccess.h"
#include "case/vehicle_run/diagnostics/Publication.h"
#include "../TwoIntervalAcceptance.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <utility>

namespace crash::cases::vehicle_run::observed {
namespace {
namespace fs = std::filesystem;

bool Within(const fs::path& path, const fs::path& directory) {
    return std::mismatch(directory.begin(), directory.end(),
                         path.begin(), path.end()).first == directory.end();
}

// This runs after the normal controller has closed. Diagnostics never alter
// its result, retry its candidate, or replace the shared acceptance assertions.
void PublishFailure(const ObservedResult& observed, const fs::path& directory) {
    ASSERT_TRUE(observed.failure);
    const auto& failure = *observed.failure;
    ::testing::Test::RecordProperty("failure_capture_seen", failure.seen() ? "true" : "false");
    const auto published=diagnostics::detail::PublishFailure(observed.run,failure,directory);
    ::testing::Test::RecordProperty("failure_capture_scope",diagnostics::FailureStatusName(published.status));
    if(!published.manifest.empty()) {
        ::testing::Test::RecordProperty("failure_manifest",published.manifest.string());
        ::testing::Test::RecordProperty("failure_manifest_sha256",published.sha256);
        std::cout << "V5_OBSERVED_CONTROLLER native_rejected=1 accepted_epoch="
                  << failure.phase().accepted_epoch << " manifest=" << published.manifest
                  << " sha256=" << published.sha256 << std::endl;
    }
    if(!failure.seen())
        EXPECT_EQ(fs::symlink_status(directory).type(),fs::file_type::not_found);
    EXPECT_NE(published.status,diagnostics::FailureStatus::CaptureIncomplete) << published.error;
    EXPECT_NE(published.status,diagnostics::FailureStatus::ExportFailed) << published.error;

}

} // namespace

TEST(VehicleRunWallSelfContactObserved,
     TwoCommittedV5IntervalsPreserveBothContactProfilesAndReplay) {
    const auto* failure_output = std::getenv("ROBO_SELF_CONTACT_FAILURE_OUTPUT");
    const auto* run_output = std::getenv("ROBO_VEHICLE_RUN_OUTPUT");
    ASSERT_TRUE(failure_output && *failure_output)
        << "Set an absent ROBO_SELF_CONTACT_FAILURE_OUTPUT directory";
    ASSERT_TRUE(run_output && *run_output)
        << "Set a pre-created empty ROBO_VEHICLE_RUN_OUTPUT directory";
    const auto requested = fs::absolute(failure_output).lexically_normal();
    ASSERT_FALSE(requested.filename().empty());
    ASSERT_EQ(fs::symlink_status(requested).type(), fs::file_type::not_found);
    ASSERT_EQ(fs::symlink_status(requested.parent_path()).type(), fs::file_type::directory);
    ASSERT_EQ(fs::symlink_status(run_output).type(), fs::file_type::directory);
    ASSERT_TRUE(fs::is_empty(run_output));
    const auto directory = fs::canonical(requested.parent_path()) / requested.filename();
    ASSERT_FALSE(Within(directory, fs::canonical(run_output)))
        << "Failure fixture must be outside the authenticated controller output";

    test::CheckTwoCommittedV5Intervals(
        [&](const PreparedRun& plan, const fs::path& destination, const Control& control) {
            auto observed = RunAccess::Execute(plan, destination, control);
            try {
                ::testing::Test::RecordProperty("observed_complete_host_reservation",
                    std::to_string(observed.reservation.host_bytes));
                ::testing::Test::RecordProperty("observed_complete_archive_reservation",
                    std::to_string(observed.reservation.archive_bytes));
                PublishFailure(observed, directory);
            } catch (const std::exception& error) {
                ADD_FAILURE() << "Native result retained; failure diagnostics threw: " << error.what();
            } catch (...) {
                ADD_FAILURE() << "Native result retained; failure diagnostics threw an unknown exception";
            }
            // Even a successfully exported rejection must fail the SAME two-
            // commit assertions. No diagnostic result can turn this into Ok.
            return std::move(observed.run);
        });
}

} // namespace crash::cases::vehicle_run::observed
