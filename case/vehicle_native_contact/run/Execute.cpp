#include "State.h"
#include "output/physical_run/ViewerInput.h"
#include "case/vehicle_dynamics/native_contact/Group.h"
#include <chrono>
namespace crash::cases::vehicle_native_contact {
RunResult PreparedRun::Execute(const std::filesystem::path& destination, const vehicle_run::Control& control) const {
    output::Require(data_->forecast.fits_runtime_limits,
                    "Complete source/owner/native/output forecast does not fit; no owner allocated");
    output::Require(std::filesystem::symlink_status(destination).type() == std::filesystem::file_type::directory &&
                        std::filesystem::is_empty(destination), "Native accepted run requires a real empty destination");
    vehicle_run::detail::ValidateLoop(data_->horizon, data_->forecast.archive.archive.archive.frame_epochs, control);
    std::filesystem::create_directory(destination / "archive");
    using Clock = std::chrono::steady_clock;
    const auto seconds = [](Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); };
    RunResult result;
    std::unique_ptr<Session> session;
    const auto start = Clock::now();
    try {
        session = std::make_unique<Session>(*data_, destination / "archive");
        result.session_startup_s = seconds(start);
        result.session_initialized = true;
        const auto* group = session->dynamics.native_contact_group();
        for (std::size_t i = 0; i < result.initialization.size(); ++i) {
            result.initialization[i] = group->transaction(i).initialization_diagnostics();
            output::Require(result.initialization[i].available, "Actual native interface lacks successful source initialization diagnostics");
        }
        if (data_->config.verify_initial_retry) {
            const auto retry = Clock::now();
            try { session->VerifyInitialRetry(); }
            catch (...) { result.retry_probe_s = seconds(retry); throw; }
            result.retry_probe_s = seconds(retry);
            result.initial_retry_verified = true;
        }
    } catch (const std::exception& error) {
        if (!result.session_initialized) result.session_startup_s = seconds(start);
        result.loop.kind = vehicle_run::StopKind::StartupFailure;
        result.loop.reason = std::string(error.what()).substr(0, 4096);
        if (session) {
            result.loop.progress.accepted = session->Accepted();
            result.rejected_native = session->rejected_native;
            result.rejected_step_limit_s = session->rejected_step_limit_s;
        }
        try { result.summary = run_detail::WriteSummary(destination, data_->source, data_->config,
            data_->forecast, data_->horizon, result); }
        catch (const std::exception& failure) { result.summary_error = failure.what(); }
        return result;
    }
    const auto loop_start = Clock::now();
    const auto clock = [&] { return seconds(loop_start); };
    result.loop = vehicle_run::detail::RunLoop(*session, data_->horizon,
        session->archive.forecast().archive.archive.frame_epochs, control, clock);
    if (const auto* captured = session->dynamics.qeph_rejection())
        result.qeph_rejection = vehicle_dynamics::diagnostics::qeph_rejection::ExportForRun(
            *captured, destination);
    result.native = session->native;
    result.archive_manifest = session->manifest;
    result.rejected_native = session->rejected_native;
    result.rejected_step_limit_s = session->rejected_step_limit_s;
    if (result.archive_manifest) {
        try {
            result.viewer_input = output::physical_run::WriteViewerInput(destination, "viewer-input.json",
                {"archive", *result.archive_manifest, data_->mapping.source_mapping().source().data().inputs,
                 data_->mapping.source_mapping().digest()});
        } catch (const std::exception& error) { result.viewer_error = std::string(error.what()).substr(0, 4096); }
    }
    try { result.summary = run_detail::WriteSummary(destination, data_->source, data_->config,
        data_->forecast, data_->horizon, result); }
    catch (const std::exception& error) { result.summary_error = std::string(error.what()).substr(0, 4096); }
    return result;
}
} // namespace crash::cases::vehicle_native_contact
