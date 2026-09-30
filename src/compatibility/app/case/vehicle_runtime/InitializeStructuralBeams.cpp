#include "Storage.h"
#include "BeamRuntime.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InitializeStructuralBeams() {
    const auto* model=source.structural_beams();
    if(!model) return;
    auto next=std::make_unique<tl::fea::beam18::Batch>();
    detail::RequireSuccess(next->InitializeJoined(
        detail::ConfigureStructuralBeams(config,source,owner.accepted()),*model));
    output::Require(next->allocations().device_bytes==forecast.structural_beams.device_bytes &&
        next->startup_host_bytes()<=forecast.structural_beams.startup_host_bytes,
        "Actual structural beam allocation differs from the complete forecast");
    beam18=std::move(next);
}
} // namespace crash::cases::vehicle_runtime
