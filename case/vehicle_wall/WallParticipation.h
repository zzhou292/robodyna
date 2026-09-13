#pragma once
#include "VehicleWallStartup.h"
namespace crash::cases::vehicle_wall::detail {
tl::fea::ShellPhysicalScratchParticipationForecast ForecastWallParticipation(
    std::uint64_t,const tl::fea::ShellPhysicalScratchParticipationLimits&);
} // namespace crash::cases::vehicle_wall::detail
