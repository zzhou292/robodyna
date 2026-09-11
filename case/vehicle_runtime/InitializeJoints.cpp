#include "Storage.h"
#include "JointRuntime.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InitializeJoints() {
    if(!joint_model) return;
    detail::CheckJointSource(execution,*joint_model);
    auto next=std::make_unique<tl::fea::type45::Batch>();
    detail::RequireSuccess(next->InitializeJoined(
        detail::ConfigureJoints(config,attachments,owner.accepted()),joint_model->model()));
    output::Require(next->allocations().device_bytes==forecast.joints.device_bytes &&
        next->startup_host_bytes()<=forecast.joints.startup_host_bytes,
        "Actual native joint allocation differs from the complete forecast");
    type45=std::move(next);
}
} // namespace crash::cases::vehicle_runtime
