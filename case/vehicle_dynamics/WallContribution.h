#pragma once
#include "WallObservation.h"
namespace crash::cases::vehicle_wall {
class VehicleWallSetup;
struct RuntimeForecast;
}
namespace crash::cases::vehicle_dynamics::detail {
// One private optional wall seam, installed only by the authenticated loaded
// factory. No public registry, owner injection, publication or second clock.
// The concrete wall module owns its own scratch and exact immutable setup.
class WallContribution {
  public:
    virtual ~WallContribution()=default;
    virtual void Assemble(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
        const tl::fea::NodalAssemblyView&,WallObservation&)=0;
    virtual void Evaluate(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
        const tl::fea::NodalPreparedView&,const tl::fea::ShellPhysicalDiagnostics&,WallObservation&)=0;
    virtual void Discard() noexcept=0;
    virtual const vehicle_wall::VehicleWallSetup& setup() const noexcept=0;
    virtual const vehicle_wall::RuntimeForecast& forecast() const noexcept=0;
    virtual tl::fea::NodalAllocationInfo allocations() const noexcept=0;
};
} // namespace crash::cases::vehicle_dynamics::detail
