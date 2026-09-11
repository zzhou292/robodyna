#pragma once
#include "VehicleType25Source.h"
#include "modelio/physical_scope/CanonicalDomain.h"
#include "modelio/source_assembly/ResolvedSpotweldProperty.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::type25::detail {
using output::Require;
void Add(std::size_t&, std::size_t count, std::size_t width, std::size_t cap);
using physical_scope::detail::CheckDomain;
std::vector<native::ConnectionInput> Pack(const std::vector<physical_scope::Spotweld>&,
                                        const tl::fea::NodalNodeDomain&);
} // namespace crash::modelio::type25::detail
