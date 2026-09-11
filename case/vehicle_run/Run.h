#pragma once
#include "Config.h"
#include "Progress.h"
#include "case/vehicle_wall/LoadedWall.h"
#include "case/vehicle_dynamics/output/VehicleAcceptedFrames.h"
#include "output/physical_run/RunArchive.h"
#include "output/physical_run/ViewerInput.h"
#include <optional>
namespace crash::cases::vehicle_run {
namespace records=output::full_shell;
inline constexpr std::size_t SummaryByteCap=1u<<20;
inline constexpr std::size_t CompanionByteCap=SummaryByteCap+output::physical_run::ViewerInputByteCap;
struct Forecast {
    vehicle_wall::RuntimeForecast wall;
    output::physical_frames::Forecast capture;
    output::physical_run::Forecast archive;
    ResourceCaps caps;
    std::size_t complete_host_bytes=0,complete_archive_bytes=0;
};
struct Result {
    bool session_initialized=false;
    double startup_wall_s=0;
    LoopResult loop;
    vehicle_dynamics::StepTimingSnapshot mechanics_timing;
    std::optional<records::RecordFile> archive_manifest,viewer_input,summary;
    std::optional<double> rejected_step_limit_s;
    std::uint32_t rejected_node=UINT32_MAX,rejected_parent=UINT32_MAX;
    std::optional<tlfea::contact::NodalWallDeviceStatus> rejected_contact_status;
    std::string viewer_input_error,summary_error;
};
// Source-bound preflight and run orchestration only. Physics always belongs to
// the existing LoadedWall/Dynamics owner; visualization is not restart state.
class PreparedRun {
  public:
    static PreparedRun Prepare(const vehicle_wall::VehicleWallSetup&,const vehicle_runtime::JointModel&,
        Config,records::Identity);
    const Forecast& forecast() const noexcept;
    const Horizon& horizon() const noexcept;
    Result Execute(const std::filesystem::path& empty_destination,const Control& = {}) const;
  private:
    struct Data;
    struct Session;
    explicit PreparedRun(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_run
