#pragma once
#include "VehicleType45Source.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::type45::detail {
inline physical_domain::Policy DomainPolicy(Policy policy) {
    if (policy == Policy::OriginalDirectSdiType45V1) return physical_domain::Policy::RetainedShellAssembliesV1;
    if (policy == Policy::OriginalDirectSdiType45VehicleSupportsV5)
        return physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5;
    output::Require(policy == Policy::OriginalDirectSdiType45ExtendedSolidsV4, "Unsupported original TYPE45 source policy");
    return physical_domain::Policy::RetainedShellAssembliesExtendedSolidsV4;
}
inline std::size_t Required(Policy policy) {
    (void)DomainPolicy(policy);
    return policy == Policy::OriginalDirectSdiType45VehicleSupportsV5 ? 44 :
        policy == Policy::OriginalDirectSdiType45ExtendedSolidsV4 ? 40 : 38;
}
inline bool Boundary(Policy policy, std::uint64_t id) {
    (void)DomainPolicy(policy);
    if (policy == Policy::OriginalDirectSdiType45VehicleSupportsV5) return false;
    return (id >= 2200526 && id <= 2200529) ||
        (policy == Policy::OriginalDirectSdiType45V1 && (id == 2200514 || id == 2200515));
}
} // namespace crash::modelio::type45::detail
