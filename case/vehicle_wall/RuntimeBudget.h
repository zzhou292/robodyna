#pragma once
#include "VehicleWallStartup.h"
namespace crash::cases::vehicle_wall::detail {
// Call only after proving identical retained source handles. Previous source
// construction ceilings are not live allocations. Scratch phases do not sum.
RuntimeForecast ComposeForecast(const vehicle_dynamics::Forecast&, const SetupForecast&,
    const tl::fea::ShellMappedFootprint&, std::size_t fixed_bytes, RuntimeLimits);
} // namespace crash::cases::vehicle_wall::detail
