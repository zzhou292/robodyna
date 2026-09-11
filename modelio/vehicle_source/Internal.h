#pragma once
#include "VehicleSourcePlan.h"
#include "SourceCards.h"
#include "modelio/source_assembly/JsonReader.h"
#include <map>

namespace crash::modelio::vehicle::detail {
using namespace assembly::reader;
struct Declarations {
    // Reuses the existing declaration value codec; geometry/state stay empty.
    assembly::Data typed;
    std::vector<PartDisposition> parts;
    std::vector<std::size_t> materials, sections;
};
struct Geometry {
    std::vector<ParentIndex> parents;
    std::vector<std::uint32_t> nodes;
    Counts counts;
};
std::size_t Preflight(const source::CanonicalData&,const assembly::ArtifactIdentity&,Limits);
void CheckAuthority(const source::CanonicalData&,const Value&);
Declarations ReadDeclarations(const source::CanonicalData&,const Value&,Limits);
Geometry ReadGeometry(const source::CanonicalData&,const std::vector<PartDisposition>&);
} // namespace crash::modelio::vehicle::detail
