#pragma once
#include "Types.h"
namespace crash::output::full_shell::source {
enum class MappingExecutionProfile { LegacyOpaque, NativeA62OrdinaryLaw1 };
struct MappingExecutionPart {
    std::uint64_t part = 0;
    std::uint64_t material = 0;
    std::uint64_t section = 0;
    std::uint64_t qeph = 0;
    std::uint64_t t3 = 0;
};
// Descriptive, source-consistent execution provenance. The physical factory
// supplies resolved roles; this I/O layer never authorizes mechanics or parses
// material cards. Eight binary mapping arrays and frame codecs are unchanged.
struct MappingExecution {
    MappingExecutionProfile profile = MappingExecutionProfile::LegacyOpaque;
    double projection_working_length_m = 0;
    double coefficient_working_length_m = 0;
    std::vector<MappingExecutionPart> parts; // Complete selected global-law1 PIDs, sorted.
};
namespace detail {
void CheckMappingExecution(const MappingExecution&);
Document MappingExecutionDocument(const MappingExecution&);
MappingExecution ParseMappingExecution(const Value&);
std::string MappingExecutionDigest(const std::string& arrays_digest, const MappingExecution&);
}
}
