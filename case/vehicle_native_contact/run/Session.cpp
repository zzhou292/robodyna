#include "State.h"
#include "ArchiveRequest.h"
#include "case/vehicle_run/MechanicsSummary.h"
#include "case/vehicle_run/SampledShellPlasticity.h"
#include "case/vehicle_dynamics/native_contact/Group.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact {
PreparedRun::Session::Session(const Data& data, const std::filesystem::path& directory)
    : source(data), dynamics(data.source.Initialize()), capture(data.mapping, dynamics, data.config.identity, data.config.capture),
      archive(output::physical_run::RunArchive::PrepareWithEnvironment(directory, data.mapping, capture.frames().context(),
          run_detail::MakeArchiveRequest(capture.frames().context(), data.horizon, data.config.samples,
              data.config.archive_bytes, data.config.artifact_file_bytes), data.profile, data.config.archive)) {
    const auto* group = dynamics.native_contact_group();
    output::Require(group && group->count() == native.size(), "Native run requires both actual source interfaces");
    for (std::size_t i = 0; i < native.size(); ++i) {
        native[i].role = group->role(i);
        native[i].source_id = group->transaction(i).source_info().source_id;
    }
}
vehicle_run::Endpoint PreparedRun::Session::Accepted() const noexcept {
    const auto stamp = dynamics.accepted();
    return {stamp.epoch, stamp.time};
}
const vehicle_dynamics::StepObservation& PreparedRun::Session::PrepareObserved() {
    try { return dynamics.PrepareStep(); }
    catch (const vehicle_dynamics::native_contact::StageError& error) {
        rejected_native = error.failure();
        throw;
    } catch (const vehicle_dynamics::StepSizeError& error) {
        rejected_step_limit_s = error.limit();
        throw;
    }
}
void PreparedRun::Session::Prepare() { (void)PrepareObserved(); }
void PreparedRun::Session::Commit() { dynamics.CommitStep(); }
void PreparedRun::Session::Discard() noexcept { dynamics.DiscardStep(); }
void PreparedRun::Session::Append() {
    const auto row = output::physical_run::CaptureAcceptedInterval(dynamics, capture.frames(), source.profile);
    const auto& step = dynamics.last_accepted_step();
    auto staged_mechanics = mechanics;
    vehicle_run::ObserveAcceptedMechanics(staged_mechanics, step, dynamics.accepted());
    auto staged_native = native;
    output::Require(row.values().native_group && row.values().native_group->count == staged_native.size(),
                    "Accepted native output lacks the actual interface group");
    for (std::size_t i = 0; i < staged_native.size(); ++i) {
        auto& totals = staged_native[i];
        const auto& values = step.native_contact.interfaces[i];
        output::Require(values.enabled && values.source.source_id == totals.source_id &&
                            totals.intervals + 1 == dynamics.accepted().epoch && step.native_contact.roles[i] == totals.role,
                        "Native counters must follow the authenticated next accepted interface");
        ++totals.intervals;
        const auto& counters = values.diagnostics;
        totals.peak_active_forces = std::max(totals.peak_active_forces, counters.active_forces);
        totals.peak_raw_candidates = std::max(totals.peak_raw_candidates, counters.raw_candidates);
        totals.peak_optimized_candidates = std::max(totals.peak_optimized_candidates, counters.optimized_candidates);
        if (counters.active_forces) {
            ++totals.active_force_intervals;
            if (!totals.first_active_epoch) {
                totals.first_active_epoch = dynamics.accepted().epoch;
                totals.first_active_time_s = dynamics.accepted().time;
            }
        }
    }
    archive.Append(row);
    mechanics = staged_mechanics;
    native = staged_native;
}
void PreparedRun::Session::Capture() { capture.Capture(dynamics); }
void PreparedRun::Session::SaveSample() {
    const auto& frames = capture.frames();
    output::Require(frames.frame() && frames.activity(), "Sample has no authentic accepted frame/activity");
    vehicle_run::detail::SaveShellSample(plasticity, frames.context(), *frames.frame(),
        [&] { archive.Sample(*frames.frame(), *frames.activity()); });
}
void PreparedRun::Session::Finish(bool complete, const std::string& reason) {
    manifest = complete ? archive.Finish() : archive.FinishPrefix(reason);
}
} // namespace crash::cases::vehicle_native_contact
