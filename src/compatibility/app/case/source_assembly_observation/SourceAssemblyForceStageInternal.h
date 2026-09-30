#pragma once
#include "SourceAssemblyForceStageKinetic.h"
#include "SourceAssemblyKineticChannels.h"
#include "lib_src/constraints/NodalRigidForceStageKinetic.h"

namespace crash::cases::source_assembly_observation::detail {
Report CheckForceStage(const ForceStageInput&,ForceStageSummary*,
                       rigid::ForceStageObservationPhase&) noexcept;
} // namespace crash::cases::source_assembly_observation::detail
