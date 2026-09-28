#pragma once
#include "VehicleContactStartup.h"
#include "case/vehicle_run/Progress.h"
#include "case/vehicle_dynamics/native_contact/Error.h"
#include "output/physical_run/RunArchive.h"
#include "output/physical_frames/PhysicalAcceptedFrames.h"
#include "case/vehicle_dynamics/diagnostics/qeph_rejection/Export.h"
namespace crash::cases::vehicle_native_contact {
namespace records = output::full_shell;
struct RunConfig {
    std::size_t samples = 31;
    std::size_t archive_bytes = records::TotalByteCap;
    // Per-artifact bound also controls interval and original-source chunking.
    std::size_t artifact_file_bytes = output::kArtifactFileCap;
    std::size_t mapping_bytes = 512u << 20;
    output::physical_frames::Limits capture;
    output::physical_run::Limits archive;
    records::Identity identity;
    bool verify_initial_retry = false; // Qualification probe, before the timed loop.
};
struct RunForecast {
    output::physical_frames::Forecast capture;
    output::physical_run::Forecast archive;
    std::size_t controller_bytes = 0, retry_bytes = 0, mapping_bytes = 0;
    std::size_t preparation_peak_host_bytes = 0, complete_peak_host_bytes = 0;
    bool fits_runtime_limits = false;
};
struct NativeTotals {
    Role role = Role::Self;
    std::uint64_t source_id = 0, intervals = 0, active_force_intervals = 0;
    std::uint64_t peak_active_forces = 0, peak_raw_candidates = 0, peak_optimized_candidates = 0;
    std::uint64_t first_active_epoch = 0;
    double first_active_time_s = 0;
};
struct RunResult {
    vehicle_run::LoopResult loop;
    double session_startup_s = 0, retry_probe_s = 0;
    bool session_initialized = false, initial_retry_verified = false;
    std::array<n::TransactionInitializationDiagnostics, 2> initialization;
    std::array<NativeTotals, 2> native;
    std::optional<vehicle_dynamics::native_contact::Failure> rejected_native;
    std::optional<double> rejected_step_limit_s;
    std::optional<vehicle_dynamics::diagnostics::qeph_rejection::ExportResult> qeph_rejection;
    std::optional<records::RecordFile> archive_manifest, viewer_input, summary;
    std::string viewer_error, summary_error;
};
// Thin accepted-output/controller adapter. Source preparation remains a separate
// timed caller stage. Execute uses the existing RunLoop and one actual owner.
class PreparedRun {
  public:
    static PreparedRun Prepare(const VehicleContactStartup&, RunConfig);
    const RunForecast& forecast() const noexcept;
    const output::physical_frames::Mapping& mapping() const noexcept;
    RunResult Execute(const std::filesystem::path& empty_destination, const vehicle_run::Control& = {}) const;
  private:
    struct Data;
    struct Session;
    explicit PreparedRun(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_native_contact
