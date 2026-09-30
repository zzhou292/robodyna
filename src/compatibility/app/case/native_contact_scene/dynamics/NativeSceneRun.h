#pragma once
#include "NativeSceneDynamics.h"
#include "../ArchiveSource.h"
#include "case/vehicle_run/Loop.h"
#include "output/physical_run/RunArchive.h"
namespace crash::cases::native_scene {
struct RunConfig {
    DynamicsConfig dynamics;
    std::uint64_t run_id=0,steps=1000;
    std::size_t samples=31,host_bytes=1u<<30,archive_bytes=2u<<30;
    output::physical_frames::Limits capture;
};
struct RunForecast {
    DynamicsForecast dynamics;
    output::physical_frames::Forecast capture;
    output::physical_run::Forecast archive;
    std::size_t host_upper_bound=0,device_upper_bound=0;
};
struct RunResult {
    vehicle_run::LoopResult loop;
    double startup_wall_s=0;
    std::optional<output::full_shell::RecordFile> manifest,viewer_input,summary;
    std::string output_error;
};
// Source/forecast preparation is host-only. The caller supplies an already
// emitted, authenticated static source; Execute alone creates the GPU owner.
class PreparedNativeSceneRun {
  public:
    static PreparedNativeSceneRun Prepare(const ContactSelection&,const ArchiveSource&,RunConfig);
    const RunForecast& forecast() const noexcept;
    const vehicle_run::Horizon& horizon() const noexcept;
    output::Document ForecastDocument() const;
    RunResult Execute(const std::filesystem::path& fresh_empty_directory,const vehicle_run::Control& = {}) const;
  private:
    struct Data;std::shared_ptr<const Data> data_;
    explicit PreparedNativeSceneRun(std::shared_ptr<const Data> p):data_(std::move(p)){}
};
}
