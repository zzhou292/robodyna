#pragma once

#include "Request.h"
#include "case/vehicle_native_contact/Run.h"
#include "case/vehicle_native_contact/source/OriginalSources.h"
#include "output/ArtifactIO.h"

namespace robodyna::driver {
namespace vehicle = crash::cases::vehicle_native_contact;
inline constexpr std::size_t ExportBytes = 2u << 20;
inline constexpr std::size_t ReplayBytes = 512u << 20;

// Retain original source authority for the complete prepared run lifetime.
// No owner is created until Execute. Source preparation itself uses CUDA.
struct Prepared {
    vehicle::source::OriginalSources originals;
    vehicle::VehicleContactStartup source;
    vehicle::PreparedRun run;
    std::size_t source_extra_bytes = 0, replay_peak_bytes = 0;
};
Prepared Prepare(const Request&);
crash::output::Document PlanDocument(const Request&, const Prepared&, const char* mode);
int Execute(const Request&, const Prepared&, crash::output::Document& report);
}  // namespace robodyna::driver
