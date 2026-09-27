#pragma once
#include "OriginalPaths.h"
#include <cstddef>
#include <string>
namespace crash::output::full_shell::source { class CanonicalSource; }
namespace crash::modelio::vehicle { class VehicleSourcePlan; class VehicleSectionResolution; }
namespace crash::cases::vehicle_run::detail {
// Bounded readers for the pinned original input profile. They authenticate
// bytes and compose existing source readers; no case or physical owner is made.
std::string ReadOriginal(const std::filesystem::path&,std::size_t,const char* sha256);
output::full_shell::source::CanonicalSource ReadCanonical(const OriginalPaths&,const std::string& member);
modelio::vehicle::VehicleSectionResolution Resolve(const modelio::vehicle::VehicleSourcePlan&,
    const OriginalPaths&,const std::string& member);
} // namespace crash::cases::vehicle_run::detail
