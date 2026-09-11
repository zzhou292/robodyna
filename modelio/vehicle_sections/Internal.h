#pragma once
#include "VehicleSectionResolution.h"
#include "modelio/source_assembly/JsonReader.h"
#include "modelio/vehicle_source/SourceCards.h"

namespace crash::modelio::vehicle::resolution {
using namespace assembly::reader;
struct Declarations {
    assembly::Data failure;
    std::vector<SectionPartResolution> parts;
    ResolutionCounts counts;
};
std::size_t Preflight(const VehicleSourcePlan&, const assembly::ArtifactIdentity&, ResolutionLimits);
void CheckAuthority(const VehicleSourcePlan&, const Value&);
Declarations ReadDeclarations(const VehicleSourcePlan&, const Value&, ResolutionLimits);
std::vector<SectionParentResolution> ReadParents(const VehicleSourcePlan&);
} // namespace crash::modelio::vehicle::resolution
