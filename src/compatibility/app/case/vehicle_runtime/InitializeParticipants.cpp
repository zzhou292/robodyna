#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InitializeParticipants() {
    const auto c = detail::ConfigureParticipants(config,source,owner.accepted());
    const auto cin = source.witness_source();
    const auto& physical = source.physical();
    detail::RequireSuccess(qeph.InitializeMapped(c.qeph,physical,owner,cin,config.limits.failure));
    detail::RequireSuccess(t3.InitializeMapped(c.t3,physical,owner,cin,config.limits.failure));
    detail::RequireSuccess(qbat.InitializeMapped(c.qbat,physical,owner,cin));
    detail::RequireSuccess(type25.InitializeMapped(c.type25,physical,owner,cin,tl::fea::type25::CapacityProfile::Vehicle));
    detail::RequireSuccess(type13.InitializeMapped(c.type13,physical,source.rigid(),
        owner,cin,config.limits.beams));
    detail::RequireSuccess(solids.InitializeJoined(c.solids,source.solids()));
    InitializeStructuralBeams();
}
} // namespace crash::cases::vehicle_runtime
