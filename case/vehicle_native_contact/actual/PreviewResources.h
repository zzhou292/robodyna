#pragma once
#include "../VehicleContactStartup.h"
namespace crash::cases::vehicle_native_contact::test {
// Explicit execution capacities for the reviewed first preview. These are
// resource ceilings, not contact selection or numerical input. Every native
// count remains complete and rejects before publication when a ceiling is
// exceeded. Product Config defaults and all physical/contact laws are unchanged.
inline Config PreviewResources() {
    Config config;
    config.dynamics.startup.limits.device_bytes = std::size_t{5} << 30;
    config.initialization[0].max_pairs = std::size_t{1} << 20;
    config.initialization[0].max_tasks = std::size_t{5} << 20;
    config.initialization[1].max_pairs = 65536;
    config.initialization[1].max_tasks = 65536;
    config.transaction[0].inventory.max_pairs = std::size_t{2} << 20;
    config.transaction[1].inventory.max_pairs = std::size_t{1} << 18;
    return config;
}
} // namespace crash::cases::vehicle_native_contact::test
