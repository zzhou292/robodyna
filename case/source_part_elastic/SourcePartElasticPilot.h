#pragma once
#include "SourcePartElasticCase.h"

namespace crash::cases::source_part_elastic {
inline constexpr double PilotStep = 0x1p-24;
inline constexpr std::uint64_t PilotHorizonSteps = 32768;
// Frozen physical experiment in planning/SOURCE_PART_ELASTIC_PILOT.md.
// Refinement changes only the integration interval, never the load or horizon.
Config PilotConfig(unsigned refinement);
}
