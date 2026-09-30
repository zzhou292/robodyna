#pragma once
#include "case/source_assembly_dynamics/StepTiming.h"
#include <filesystem>
#include <string>

namespace crash::cases::source_assembly_wall {
// Call before startup; the output path must be outside the accepted archive.
// Resolves existing parent symlinks. No directory or file is created here.
void CheckStepTimingPath(const std::filesystem::path& output,const std::filesystem::path& archive);
std::string FormatStepTiming(const source_assembly_dynamics::StepTimingSnapshot&,int execution_status);
// Create-only, checked write/close; no parent directories are created. Failure
// throws to the diagnostic caller, which must preserve the simulation result.
void WriteStepTiming(const std::filesystem::path&,const source_assembly_dynamics::StepTimingSnapshot&,int execution_status);
}
