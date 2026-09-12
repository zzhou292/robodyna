#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InitializeParticipants() {
    const auto c = detail::ConfigureParticipants(config,execution,attachments,owner.accepted());
    const auto source = detail::Witnesses(attachments);
    const auto& physical = execution.physical();
    detail::RequireSuccess(qeph.InitializeMapped(c.qeph,physical,owner,source,config.limits.failure));
    detail::RequireSuccess(t3.InitializeMapped(c.t3,physical,owner,source,config.limits.failure));
    detail::RequireSuccess(qbat.InitializeMapped(c.qbat,physical,owner,source));
    detail::RequireSuccess(type25.InitializeMapped(c.type25,physical,owner,source,tl::fea::type25::CapacityProfile::Vehicle));
    detail::RequireSuccess(type13.InitializeMapped(c.type13,physical,execution.model().rigid_assembly(),
        owner,source,config.limits.beams));
    detail::RequireSuccess(solids.InitializeJoined(c.solids,execution.model().solids()));
    InitializeStructuralBeams();
}
} // namespace crash::cases::vehicle_runtime
