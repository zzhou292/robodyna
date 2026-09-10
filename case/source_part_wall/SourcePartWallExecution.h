#pragma once
#include "case/source_part_elastic/SourcePartElasticCase.h"
#include <cstdint>
#include <string>

namespace crash::cases::source_part_wall {
std::uint64_t ParseWallCount(const char*,std::uint64_t maximum);
std::string FormatWallFailure(const source_part_elastic::Report&);
// Shared execution/output orchestration for elastic and plastic material
// choices. The caller initializes the existing case and owns its physical input.
// Returns 0 for the declared horizon, 2 for a safely closed accepted prefix.
int ExecuteWallCase(source_part_elastic::SourcePartElasticCase&,std::uint64_t steps,
    unsigned frame_every,const std::string& new_directory,std::uint64_t run_id,std::uint64_t topology_id);
}
