#include "Storage.h"
#include "JointRuntime.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InitializeJoints() {
    if(!source.joints()) return;
    auto next=std::make_unique<tl::fea::type45::Batch>();
    detail::RequireSuccess(next->InitializeJoined(
        detail::ConfigureJoints(config,source,owner.accepted()),*source.joints()));
    output::Require(next->allocations().device_bytes==forecast.joints.device_bytes &&
        next->startup_host_bytes()<=forecast.joints.startup_host_bytes,
        "Actual native joint allocation differs from the complete forecast");
    type45=std::move(next);
}
} // namespace crash::cases::vehicle_runtime
