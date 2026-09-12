#pragma once
#include <cstddef>
#include <string>
namespace crash::cases::vehicle_dynamics {
class VehiclePhysicalDynamics;
// On-demand accepted diagnostic of the actual loaded owner/source. Does not
// allocate while stepping, read CUDA, change archives or advance a clock.
// Report is complete or throws before returning: no truncated incident list.
// Temporary reservation: 2*byte_cap + (2048+CIN rows)*sizeof(size_t) + 4096.
std::string StructuralLimiterReport(const VehiclePhysicalDynamics&,
                                   std::size_t byte_cap = 1u << 20);
} // namespace crash::cases::vehicle_dynamics
