#pragma once
#include "../TiedShellSearchGeometry.h"
#include "../Internal.h"

namespace crash::modelio::tied_shell::search_detail {
using namespace assembly::reader;
struct Topology {
    std::vector<SourceId> nodes, shells;
    std::vector<std::uint32_t> connectivity;
};
struct RankedCoefficient {
    double geometry = 0, modulus = 0, part = 0, element = 0;
    SearchShellFamily family = SearchShellFamily::Q4;
};
double ConsumedThickness(const RankedCoefficient&);
double ResolveEquivalent(std::vector<RankedCoefficient> const&, std::vector<bool>& winners);
std::size_t Preflight(const source::CanonicalData&, const Data&, const PackingData&, SearchGeometryLimits);
std::size_t OwnedPayload(const SearchGeometryData&, SearchGeometryLimits);
Topology ReadTopology(const source::CanonicalData&, SearchGeometryLimits);
void Associate(const Topology&, const Data&, const PackingData&, SearchGeometryData&, SearchGeometryLimits);
void Properties(const source::CanonicalData&, const Data&, const Topology&, const std::string&,
                SearchGeometryData&, SearchGeometryLimits);
void WorkingCoordinates(const source::CanonicalData&, const Topology&, const std::string&,
                        SearchGeometryData&, SearchGeometryLimits);
void Resolve(SearchGeometryData&);
SearchGeometryData Build(const source::CanonicalData&, const Data&, const PackingData&,
                         const std::string&, SearchGeometryLimits);
template<class T> std::vector<T> Decode(const source::CanonicalData& source, const char* name) {
    const auto& a = source::FindArray(source, name);
    return output::arrays::Decode<T>(a.descriptor, a.bytes,
        {source.limits.file_bytes, std::max(source.limits.nodes, source.limits.parents), 64});
}
} // namespace crash::modelio::tied_shell::search_detail
