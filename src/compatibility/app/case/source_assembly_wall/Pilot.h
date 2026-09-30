#pragma once
#include "Execution.h"
#include "PilotOptions.h"

namespace crash::cases::source_assembly_wall {
// Explicit small integration experiment, retaining every source material and
// connection. This configuration is not a full-vehicle crash validation policy.
source_assembly_dynamics::Config PilotConfig(const PilotOptions&);
void InitializePilot(source_assembly_dynamics::SourceAssemblyWallCase&,
    const std::string& source_inventory,const std::string& original_wall,
    const output::assembly::WallArchiveRequest&,const PilotOptions& options={});
}
