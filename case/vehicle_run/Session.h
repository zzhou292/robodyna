#pragma once
#include "RunState.h"
#include "Loop.h"
namespace crash::cases::vehicle_run {
struct PreparedRun::Session final : detail::Operations {
    Session(const Data&,const std::filesystem::path& archive_directory);
    const Data& source;
    vehicle_dynamics::VehiclePhysicalDynamics dynamics;
    vehicle_dynamics::capture::VehicleAcceptedFrames capture;
    output::physical_run::RunArchive archive;
    ContactTotals contact;
    MechanicsTotals mechanics;
    SampledShellPlasticityTotals sampled_shell_plasticity;
    std::optional<records::RecordFile> manifest;
    std::optional<double> step_limit;
    std::optional<tlfea::contact::NodalWallDeviceStatus> contact_status;
    std::uint32_t node=UINT32_MAX,parent=UINT32_MAX;
    Endpoint Accepted() const noexcept override;
    ContactTotals Contact() const noexcept override {return contact;}
    MechanicsTotals Mechanics() const noexcept override {return mechanics;}
    SampledShellPlasticityTotals SampledShellPlasticity() const noexcept override {return sampled_shell_plasticity;}
    vehicle_dynamics::StepTimingSnapshot MechanicsTiming() const noexcept override {return dynamics.timing();}
    void Prepare() override;
    void Commit() override;
    void Discard() noexcept override;
    void Append() override;
    void Capture() override;
    void SaveSample() override;
    void Finish(bool,const std::string&) override;
};
} // namespace crash::cases::vehicle_run
