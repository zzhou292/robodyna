#include "Session.h"
#include "output/physical_run/Replay.h"
#include "case/vehicle_native_contact/actual/controls/PreviewControls.h"
#include <iostream>

namespace robodyna::driver {
int Execute(const Request& request, const Prepared& prepared, crash::output::Document& report) {
    namespace io = crash::output;
    namespace loop = crash::cases::vehicle_run;
    io::Require(std::filesystem::create_directory(request.output), "Accepted output already exists");
    vehicle::test::PreviewControls options;
    options.maximum_elapsed_s = request.resources.cooperative_maximum_elapsed_s;
    options.stop_file = request.stop_file;
    auto control = vehicle::test::MakePreviewControl(options);
    control.progress = [](const loop::Progress& progress) {
        std::cout << "accepted=" << progress.accepted.epoch << " time_s=" << progress.accepted.time_s
                  << " runtime_s=" << progress.elapsed_s << " steps_per_s="
                  << progress.accepted_intervals_per_second << std::endl;
    };
    report.RemoveMember("physical_owner_created");
    io::Boolean(report, "owner_creation_attempted", true);
    const auto result = prepared.run.Execute(request.output, control);
    io::Boolean(report, "session_initialized", result.session_initialized);
    io::Integer(report, "accepted_intervals", result.loop.progress.accepted.epoch);
    io::Number(report, "actual_time_s", result.loop.progress.accepted.time_s);
    io::Number(report, "runtime_elapsed_s", result.loop.progress.elapsed_s);
    io::Integer(report, "stop_kind", static_cast<unsigned>(result.loop.kind));
    io::String(report, "stop_reason", result.loop.reason);
    io::Boolean(report, "valid_closed_archive", result.loop.valid_manifest);
    const bool closed = result.session_initialized && result.loop.valid_manifest && result.summary &&
                        result.viewer_input && result.archive_manifest;
    if (!closed) return 1;
    io::Require(result.initial_retry_verified == request.run.verify_initial_retry, "Initial retry verification differs");
    const auto& mapping = prepared.run.mapping();
    const auto replay = io::physical_run::Replay::Open(request.output / "archive", *result.archive_manifest,
        mapping.source_mapping().source().data().inputs, mapping.source_mapping().digest());
    io::Require(replay.configuration().profile.native_group && !replay.configuration().profile.native_contact &&
                replay.environment() && replay.index().accepted_intervals == result.loop.progress.accepted.epoch,
                "Published native archive differs from the physical run");
    const auto sample_count = replay.index().frames.size();
    io::Require(sample_count != 0, "Closed native archive has no recorded samples");
    // ReadSample authenticates and validates frame and activity payloads. Keep
    // exactly one returned sample alive; its storage is released before the
    // next read, within the existing sequential replay memory reservation.
    for (std::size_t ordinal = 0; ordinal < sample_count; ++ordinal) {
        const auto sample = replay.ReadSample(ordinal);
        if (ordinal + 1 == sample_count)
            io::Require(sample.frame.stamp.epoch == result.loop.progress.accepted.epoch &&
                        sample.frame.stamp.time == result.loop.progress.accepted.time_s,
                        "Final replay endpoint differs");
    }
    io::Integer(report, "replay_verified_samples", sample_count);
    report["full_cpp_replay_verified"].SetBool(true);
    const bool complete = result.loop.kind == loop::StopKind::Completed && replay.index().horizon_complete &&
        result.loop.progress.accepted.epoch == prepared.run.forecast().archive.archive.archive.frame_epochs.back();
    report["horizon_complete"].SetBool(complete);
    if (complete) return 0;
    if (result.loop.kind == loop::StopKind::Requested || result.loop.kind == loop::StopKind::TimeLimit ||
        result.loop.kind == loop::StopKind::IntervalLimit) return 2;
    return 1;  // Preserve a physics-rejected archive without calling it a normal prefix completion.
}
}  // namespace robodyna::driver
