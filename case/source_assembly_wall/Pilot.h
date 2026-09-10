#pragma once
#include "Execution.h"

namespace crash::cases::source_assembly_wall {
// Explicit small integration experiment, retaining every source material and
// connection. This configuration is not a full-vehicle crash validation policy.
source_assembly_dynamics::Config PilotConfig(unsigned refinement);
void InitializePilot(source_assembly_dynamics::SourceAssemblyWallCase&,
    const std::string& source_inventory,const std::string& original_wall,
    const output::assembly::WallArchiveRequest&,unsigned refinement,
    source_assembly_dynamics::StepTimingOptions timing={});
}
