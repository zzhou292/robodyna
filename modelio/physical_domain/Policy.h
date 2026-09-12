#pragma once
#include "VehiclePhysicalDomain.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::physical_domain::detail {
inline solid_source::Policy SolidPolicy(Policy policy) {
    if (policy == Policy::RetainedShellAssembliesV1)
        return solid_source::Policy::OriginalAdhesive18RubberHephS6zV1;
    output::Require(policy == Policy::RetainedShellAssembliesExtendedSolidsV4,
                    "Unsupported physical domain source policy");
    return solid_source::Policy::OriginalExtendedSolidsV4;
}
inline bool RequiresCompleteGroup(Policy policy, std::uint64_t source_id) {
    return policy == Policy::RetainedShellAssembliesExtendedSolidsV4 &&
        ((source_id >= 2200175 && source_id <= 2200178) || source_id == 2200666 || source_id == 2200667);
}
} // namespace crash::modelio::physical_domain::detail
