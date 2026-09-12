#pragma once
#include "OriginalYaris.h"
#include "PhysicalSelection.h"
#include <optional>
#include "modelio/physical_scope/PhysicalScope.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
namespace crash::cases::vehicle_run::detail {
struct OriginalSources {
    OriginalSources(const OriginalPaths&,PhysicalProfile);
    std::string member;
    output::full_shell::source::CanonicalSource canonical;
    modelio::vehicle::VehicleSourcePlan plan;
    modelio::vehicle::VehicleSectionResolution resolution;
    modelio::physical_scope::rigid::RigidPartSource rigid;
    modelio::physical_scope::rigid::point_mass::Source masses;
    modelio::tied_shell::TiedShellDeclaration tied;
    modelio::type13::SourceType13 beams;
    modelio::solid_source::VehicleSolidSource solids;
    std::optional<modelio::beam18::Source> structural_beams;
};
std::string ReadOriginal(const std::filesystem::path&,std::size_t,const char* sha256);
}
