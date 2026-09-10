#pragma once
#include "PilotOptions.h"
#include "case/source_assembly/SourceAssemblyBindings.h"

namespace crash::cases::source_assembly_wall {
modelio::assembly::SourceAssembly ReadPilotSource(const std::string& inventory,PilotAssembly);
source_assembly::SourceAssemblyBindings PreparePilotBindings(const modelio::assembly::SourceAssembly&,PilotAssembly);
} // namespace crash::cases::source_assembly_wall
