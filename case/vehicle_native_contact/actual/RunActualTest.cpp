#include "Report.h"
#include "../Run.h"
#include "../tests/ActualSources.h"
#include "output/physical_run/ViewerInput.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
namespace crash::cases::vehicle_native_contact::test {
namespace {
constexpr std::size_t ReplayBytes = 512u << 20;
double EnvironmentReal(const char* key) {
    const char* text = std::getenv(key);
    output::Require(text && *text, "Missing explicit full native run control");
    const std::string value(text);
    std::size_t used = 0;
    const double result = std::stod(value, &used);
    output::Require(used == value.size() && std::isfinite(result) && result > 0, "Invalid explicit native run control");
    return result;
}
void RunAcceptedQualification(bool execute, bool preview) {
    const auto destination = Destination();
    auto doc = Document(execute ? "accepted_native_run" : "accepted_native_output_forecast");
    const auto begin = Clock::now();
    bool complete = !execute;
    try {
        Config config;
        config.dynamics.startup.limits.device_bytes = std::size_t{5} << 30;
        config.requested_duration_s = preview ? EnvironmentReal("ROBO_NATIVE_VEHICLE_DURATION_S") :
            2 * config.dynamics.startup.reserved_step_s;
        RunConfig run_config;
        run_config.samples = 3;
        run_config.identity.run = UINT64_C(0x4e41545635434152);
        run_config.identity.topology = UINT64_C(0x4e4154563557414c);
        run_config.verify_initial_retry = !preview;
        if (preview) {
            const double samples = EnvironmentReal("ROBO_NATIVE_VEHICLE_SAMPLES");
            output::Require(samples >= 2 && samples <= 1000 && std::floor(samples) == samples,
                            "Native preview sample count must be an integer 2..1000");
            run_config.samples = static_cast<std::size_t>(samples);
            // Explicit full-preview archive allowance. The concrete complete
            // archive forecast is inspected before the owning launch.
            run_config.archive_bytes = records::FullRunByteCap;
        }
        const auto input = ActualSources();
        output::Number(doc, "source_construction_s", Seconds(begin));
        Sources(doc, input);
        const auto early = VehicleContactStartup::ForecastPreparation(input.owner, input.self, input.wall, input.controls, config);
        output::Require(early.host_preparation_ceiling <= GuardBytes - ExportBytes,
                        "Host preparation exceeds the unchanged 18GiB qualification guard");
        const auto preparation_start = Clock::now();
        const auto source = VehicleContactStartup::Prepare(input.owner, input.self, input.wall, input.controls, config);
        output::Number(doc, "host_case_preparation_s", Seconds(preparation_start));
        Plan(doc, source.forecast(), true);
        const auto output_start = Clock::now();
        const auto run = PreparedRun::Prepare(source, run_config);
        output::Number(doc, "output_preparation_s", Seconds(output_start));
        const auto& f = run.forecast();
        output::Integer(doc, "output_preparation_peak_host_bytes", f.preparation_peak_host_bytes);
        output::Integer(doc, "run_complete_peak_host_bytes", f.complete_peak_host_bytes);
        output::Integer(doc, "capture_peak_bytes", f.capture.peak_bytes);
        output::Integer(doc, "archive_peak_host_bytes", f.archive.peak_host_bytes);
        output::Integer(doc, "archive_forecast_bytes", f.archive.archive.archive.forecast_bytes);
        output::Integer(doc, "archive_cap_bytes", run_config.archive_bytes);
        output::Boolean(doc, "run_fits_runtime_limits", f.fits_runtime_limits);
        output::Require(f.complete_peak_host_bytes <= GuardBytes - ExportBytes,
                        "Complete native run/output exceeds the unchanged 18GiB qualification guard");
        // Execute destroys the owner/capture/writer before Replay::Open. The
        // immutable case/mapping remain, and their published bounds are charged.
        const auto& base = source.forecast();
        const auto replay_phase = base.sources.retained_bytes + base.packing_retained + base.prepared_source_retained +
            run.mapping().payload_bytes() + f.controller_bytes + ReplayBytes + ExportBytes;
        output::Integer(doc, "sequential_replay_peak_host_bytes", replay_phase);
        output::Require(replay_phase <= GuardBytes, "Sequential accepted replay exceeds the qualification guard");
        if (execute) {
            const auto run_path = destination / "accepted";
            output::Require(std::filesystem::create_directory(run_path), "Accepted output already exists");
            vehicle_run::Control control;
            control.progress = [](const vehicle_run::Progress& p) {
                std::cout << "accepted=" << p.accepted.epoch << " time_s=" << p.accepted.time_s
                          << " runtime_s=" << p.elapsed_s << " steps_per_s=" << p.accepted_intervals_per_second << std::endl;
            };
            doc.RemoveMember("physical_owner_created");
            output::Boolean(doc, "owner_creation_attempted", true);
            const auto result = run.Execute(run_path, control);
            output::Boolean(doc, "physical_session_initialized", result.session_initialized);
            complete = result.loop.kind == vehicle_run::StopKind::Completed && result.loop.valid_manifest &&
                result.archive_manifest.has_value() && result.viewer_input.has_value();
            output::Integer(doc, "accepted_intervals", result.loop.progress.accepted.epoch);
            output::Number(doc, "actual_time_s", result.loop.progress.accepted.time_s);
            output::Number(doc, "session_startup_s", result.session_startup_s);
            output::Number(doc, "retry_probe_s", result.retry_probe_s);
            output::Number(doc, "runtime_elapsed_s", result.loop.progress.elapsed_s);
            output::Integer(doc, "stop_kind", static_cast<unsigned>(result.loop.kind));
            output::String(doc, "stop_reason", result.loop.reason);
            output::Boolean(doc, "initial_retry_verified", result.initial_retry_verified);
            output::Boolean(doc, "valid_closed_archive", result.loop.valid_manifest);
            EXPECT_TRUE(result.session_initialized) << result.loop.reason;
            EXPECT_EQ(result.initial_retry_verified, run_config.verify_initial_retry);
            EXPECT_TRUE(result.summary.has_value()) << result.summary_error;
            EXPECT_TRUE(result.viewer_input.has_value()) << result.viewer_error;
            EXPECT_TRUE(result.archive_manifest.has_value());
            if (result.archive_manifest) {
                const auto replay = output::physical_run::Replay::Open(run_path / "archive", *result.archive_manifest,
                    run.mapping().source_mapping().source().data().inputs, run.mapping().source_mapping().digest());
                EXPECT_TRUE(replay.configuration().profile.native_group);
                EXPECT_FALSE(replay.configuration().profile.native_contact);
                EXPECT_TRUE(replay.environment());
                EXPECT_EQ(replay.context().nodes(), 359785u);
                EXPECT_EQ(replay.context().parents().size(), 349645u);
                EXPECT_EQ(replay.index().accepted_intervals, result.loop.progress.accepted.epoch);
                const auto final = replay.ReadSample(replay.index().frames.size() - 1);
                EXPECT_EQ(final.frame.stamp.epoch, result.loop.progress.accepted.epoch);
                if (!preview) {
                    EXPECT_TRUE(replay.index().horizon_complete);
                    EXPECT_EQ(replay.index().frames.size(), 3u);
                }
            }
            // A closed diagnostic prefix is preserved on rejection, but it
            // cannot pass the requested complete-horizon delivery gate.
            EXPECT_EQ(result.loop.kind, vehicle_run::StopKind::Completed) << result.loop.reason;
            EXPECT_EQ(result.loop.progress.accepted.epoch, f.archive.archive.archive.frame_epochs.back());
            if (!preview) {
                EXPECT_EQ(result.loop.progress.accepted.epoch, 2u);
                for (const auto& one : result.native) EXPECT_EQ(one.intervals, 2u);
            }
        }
        output::Boolean(doc, "completed", complete);
    } catch (const std::exception& error) {
        Failure(doc, error);
        output::Number(doc, "total_elapsed_s", Seconds(begin));
        output::WriteJson(destination / "case.json", doc);
        throw;
    }
    output::Number(doc, "total_elapsed_s", Seconds(begin));
    output::WriteJson(destination / "case.json", doc);
}
}
TEST(NativeVehicleCaseActual, CompleteOutputForecastBeforeTheOwner) { RunAcceptedQualification(false, false); }
TEST(NativeVehicleCaseActual, TwoNativeInterfacesRetryAndPublishTwoIntervalsWithClosedArchive) { RunAcceptedQualification(true, false); }
TEST(NativeVehicleCaseActual, ExplicitPreviewOutputForecastBeforeTheOwner) { RunAcceptedQualification(false, true); }
TEST(NativeVehicleCaseActual, ExplicitPreviewHorizonUsesTheExistingRunLoopAndAcceptedArchive) { RunAcceptedQualification(true, true); }
} // namespace crash::cases::vehicle_native_contact::test
