#pragma once
#include "PhysicalScope.h"
#include "output/BoundedArrayIO.h"
#include "output/ArtifactIO.h"
#include <algorithm>

namespace crash::modelio::physical_scope::detail {
using output::Require;
template<class T> std::vector<T> Decode(const source::CanonicalData& data, const char* name) {
    const auto& array = source::FindArray(data, name);
    return output::arrays::Decode<T>(array.descriptor, array.bytes);
}
void Add(std::size_t&, std::size_t count, std::size_t width, std::size_t cap);
std::size_t NodeIndex(const std::vector<SourceId>&, SourceId);
void Mark(std::vector<std::uint16_t>&, const std::vector<SourceId>&, SourceId, Role);
void Classify(Group&);
std::vector<Spotweld> ReadSpotwelds(const std::vector<tied_shell::SourceEvidence>&, Limits);
void BuildGroups(const rigid::RigidPartSource&, const tied_shell::TiedShellDeclaration&,
                 const std::vector<SourceId>&, Data&, Limits);
void BuildRoles(const rigid::point_mass::Source&, const type13::SourceType13&,
                const solid_source::VehicleSolidSource&, const std::vector<SourceId>&, Data&);
void BuildEvidence(const source::CanonicalData&, const solid_source::VehicleSolidSource&,
                   const std::vector<SourceId>&, Data&, Limits);
std::size_t OwnedPayload(const Data&, Limits);
} // namespace crash::modelio::physical_scope::detail
