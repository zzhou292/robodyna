#pragma once
#include "case/source_assembly_dynamics/Case.h"
#include "output/source_assembly/SourceAssemblyWallSchema.h"
#include <string>
namespace crash::output::assembly {class QephSpinTrace;}

namespace crash::cases::source_assembly_wall {
std::string FailureText(const source_assembly_dynamics::Report&);
// Caller supplies one initialized case and explicit archive request. The loop
// owns no mechanics, timestep, force computation or accepted-state selector.
// Return 0 for the requested horizon, 2 for a recorded accepted prefix.
int Execute(source_assembly_dynamics::SourceAssemblyWallCase&,
            const output::assembly::WallArchiveRequest&,const std::string& new_directory,
            output::assembly::QephSpinTrace* spin_trace=nullptr);
}
