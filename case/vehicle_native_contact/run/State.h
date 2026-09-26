#pragma once
#include "../Run.h"
#include "case/vehicle_run/Loop.h"
#include "case/vehicle_dynamics/output/VehicleAcceptedFrames.h"
namespace crash::cases::vehicle_native_contact {
struct PreparedRun::Data {
    Data(const VehicleContactStartup& c, RunConfig options, output::physical_frames::Mapping m)
        : source(c), config(std::move(options)), mapping(std::move(m)) {}
    VehicleContactStartup source;
    RunConfig config;
    output::physical_frames::Mapping mapping;
    vehicle_run::Horizon horizon;
    output::physical_run::Profile profile;
    RunForecast forecast;
};
struct PreparedRun::Session final : vehicle_run::detail::Operations {
    Session(const Data&, const std::filesystem::path&);
    const Data& source;
    vehicle_dynamics::VehiclePhysicalDynamics dynamics;
    vehicle_dynamics::capture::VehicleAcceptedFrames capture;
    output::physical_run::RunArchive archive;
    vehicle_run::MechanicsTotals mechanics;
    vehicle_run::SampledShellPlasticityTotals plasticity;
    std::array<NativeTotals, 2> native;
    std::optional<records::RecordFile> manifest;
    std::optional<vehicle_dynamics::native_contact::Failure> rejected_native;
    std::optional<double> rejected_step_limit_s;
    vehicle_run::Endpoint Accepted() const noexcept override;
    vehicle_run::MechanicsTotals Mechanics() const noexcept override { return mechanics; }
    vehicle_run::SampledShellPlasticityTotals SampledShellPlasticity() const noexcept override { return plasticity; }
    vehicle_dynamics::StepTimingSnapshot MechanicsTiming() const noexcept override { return dynamics.timing(); }
    void Prepare() override;
    void Commit() override;
    void Discard() noexcept override;
    void Append() override;
    void Capture() override;
    void SaveSample() override;
    void Finish(bool, const std::string&) override;
    void VerifyInitialRetry();
};
namespace run_detail {
std::size_t Add(std::size_t, std::size_t);
records::RecordFile WriteSummary(const std::filesystem::path&, const VehicleContactStartup&,
    const RunConfig&, const RunForecast&, const vehicle_run::Horizon&, const RunResult&);
}
} // namespace crash::cases::vehicle_native_contact
