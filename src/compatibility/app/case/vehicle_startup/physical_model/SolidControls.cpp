#include "Internal.h"
namespace crash::cases::vehicle_startup::physical_model::detail {
void CheckControlBinding(const modelio::physical_domain::VehiclePhysicalDomain& domain,
    const modelio::solid_control_packets::NativePacketSource* packets) {
    const bool required=domain.policy()==modelio::physical_domain::Policy::RetainedShellAssembliesNativeSupportsV6;
    Require(required==bool(packets),"V6 mechanics requires explicit source-declared solid packets; legacy profiles remain unchanged");
    if(!packets)return;
    Require(&packets->solid_source().data()==&domain.source().solid_source().data(),
        "Native packets and mechanical model do not share the same prepared solid source");
    const auto input=packets->InputFor(domain.domain().source_instance_id());
    Require(input.profile==fe::solids::control::Profile::SourceDeclared&&
        input.parents.size()==domain.source().solid_source().data().rows.size(),
        "Native packet input and complete mechanical source population differ");
}
} // namespace crash::cases::vehicle_startup::physical_model::detail
