#pragma once
#include "VehicleWallSetup.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall::detail {
inline void CheckSharedSource(const VehicleWallSetup& setup,
    const vehicle_runtime::Execution& execution, const vehicle_runtime::Attachments& attachments) {
    // Getter reference addresses name immutable owning storage itself. Copies
    // share this authority; independently rebuilt equal values do not.
    output::Require(&setup.execution().physical() == &execution.physical() &&
        &setup.attachments().witnesses() == &attachments.witnesses(),
        "Wall setup must retain the exact execution and attachment handles of this dynamics owner");
}
} // namespace crash::cases::vehicle_wall::detail
