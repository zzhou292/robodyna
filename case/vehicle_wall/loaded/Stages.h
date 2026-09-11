#pragma once
#include "../LoadedWall.h"
#include "case/vehicle_dynamics/WallContribution.h"
namespace crash::cases::vehicle_wall {
struct LoadedWall::Stages final : vehicle_dynamics::detail::WallContribution {
    Stages(VehicleWallStartup&& value,RuntimeForecast budget,std::uint64_t intervals)
        : contact(std::move(value)),budget(budget),planned_intervals(intervals) {}
    VehicleWallStartup contact;
    RuntimeForecast budget;
    const std::uint64_t planned_intervals;
    void Assemble(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
        const tl::fea::NodalAssemblyView&,vehicle_dynamics::WallObservation&) override;
    void Evaluate(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
        const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&,
        vehicle_dynamics::WallObservation&) override;
    void Discard() noexcept override;
    const VehicleWallSetup& setup() const noexcept override { return contact.setup(); }
    const RuntimeForecast& forecast() const noexcept override { return budget; }
    tl::fea::NodalAllocationInfo allocations() const noexcept override { return contact.contact_allocations(); }
};
} // namespace crash::cases::vehicle_wall
