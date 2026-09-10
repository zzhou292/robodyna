#pragma once
#include "SourcePartElasticCase.h"

namespace crash::cases::source_part_elastic {
inline constexpr double PilotStep = 0x1p-24;
inline constexpr std::uint64_t PilotHorizonSteps = 32768;
// Frozen physical experiment in planning/SOURCE_PART_ELASTIC_PILOT.md.
// Refinement changes only the integration interval, never the load or horizon.
Config PilotConfig(unsigned refinement);
// Source-specific startup/free-flight qualification only: 1 m/s global X,
// zero external load, original geometry/mass and the same elastic override.
Config UniformFlightConfig(unsigned refinement);
}
