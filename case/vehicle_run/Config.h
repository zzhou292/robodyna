#pragma once
#include "ContactProfile.h"
#include <cstddef>
#include <cstdint>
namespace crash::cases::vehicle_run {
enum class ResourceProfile { Normal, ConditionalExpandedFull };
enum class PhysicalProfile { RetainedShellAssembliesV1, ExtendedSolidsV4, VehicleSupportsV5 };
const char* PhysicalProfileName(PhysicalProfile);
struct Config {
    // Explicit 0.5 ms preview or 5/20/50 ms run; all use the same fixed-step planner.
    double duration_s=.005;
    double fixed_dt_s=3e-7;
    std::size_t samples=101;
    ResourceProfile resources=ResourceProfile::Normal;
    PhysicalProfile physical_profile=PhysicalProfile::RetainedShellAssembliesV1;
    ContactProfile contact_profile=ContactProfile::WallOnly;
    bool self_contact_diagnostics=false;
    bool self_contact_cuda_facet_filters=false;
};
struct Horizon {
    std::uint64_t intervals=0;
    double requested_duration_s=0,fixed_dt_s=0;
    // Descriptive mathematical endpoint only, never a saved owner timestamp.
    long double nominal_endpoint_s=0;
};
Horizon Plan(const Config&);
struct ResourceCaps {std::size_t host_bytes=0,archive_bytes=0;bool expanded=false;};
ResourceCaps SelectCaps(ResourceProfile,std::size_t required_host,std::size_t required_archive);
} // namespace crash::cases::vehicle_run
