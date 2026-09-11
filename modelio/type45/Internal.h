#pragma once
#include "VehicleType45Source.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::type45::detail {
using output::Require;
using Evidence = tied_shell::SourceEvidence;
struct BodyMember { std::uint64_t node = 0; SourceBody body; };
void Add(std::size_t&, std::size_t count, std::size_t width, std::size_t cap);
bool IsJoint(const std::string&);
std::vector<Row> Read(const std::vector<Evidence>&, Limits);
void ResolveProperties(Data&);
std::vector<BodyMember> Members(const physical_domain::VehiclePhysicalDomain&);
void Map(std::vector<Row>&, const std::vector<std::uint64_t>&, const std::vector<double>&,
         const tl::fea::NodalNodeDomain&, const std::vector<BodyMember>&);
void CheckOriginal(Data&);
} // namespace crash::modelio::type45::detail
