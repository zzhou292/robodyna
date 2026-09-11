#pragma once
#include "VehicleType25Source.h"
#include "modelio/source_assembly/ResolvedSpotweldProperty.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::type25::detail {
using output::Require;
void Add(std::size_t&, std::size_t count, std::size_t width, std::size_t cap);
void CheckDomain(const std::vector<std::uint64_t>&, const std::vector<double>&,
                 const std::vector<std::uint16_t>&, const tl::fea::NodalNodeDomain&,
                 std::uint64_t source_instance, std::size_t expected_count);
std::vector<native::ConnectionInput> Pack(const std::vector<physical_scope::Spotweld>&,
                                        const tl::fea::NodalNodeDomain&);
} // namespace crash::modelio::type25::detail
