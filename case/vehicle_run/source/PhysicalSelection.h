#pragma once
#include "../Config.h"
#include "modelio/physical_domain/Policy.h"
#include "modelio/type45/SourcePolicy.h"
namespace crash::cases::vehicle_run::detail {
struct PhysicalSelection {
    modelio::solid_source::Policy solids;
    modelio::physical_domain::Policy domain;
    modelio::type45::Policy joints;
    bool extended=false,structural_beams=false;
};
inline PhysicalSelection SelectPhysical(PhysicalProfile profile) {
    using D=modelio::physical_domain::Policy;
    using J=modelio::type45::Policy;
    D domain;
    J joints;
    switch(profile) {
        case PhysicalProfile::RetainedShellAssembliesV1:
            domain=D::RetainedShellAssembliesV1; joints=J::OriginalDirectSdiType45V1; break;
        case PhysicalProfile::ExtendedSolidsV4:
            domain=D::RetainedShellAssembliesExtendedSolidsV4; joints=J::OriginalDirectSdiType45ExtendedSolidsV4; break;
        case PhysicalProfile::VehicleSupportsV5:
            domain=D::RetainedShellAssembliesVehicleSupportsV5; joints=J::OriginalDirectSdiType45VehicleSupportsV5; break;
        default: throw std::invalid_argument("Unknown Yaris physical source profile");
    }
    return {modelio::physical_domain::detail::SolidPolicy(domain),domain,joints,
        profile!=PhysicalProfile::RetainedShellAssembliesV1,profile==PhysicalProfile::VehicleSupportsV5};
}
} // namespace crash::cases::vehicle_run::detail
