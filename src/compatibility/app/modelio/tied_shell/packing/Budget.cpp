#include "Internal.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::tied_shell::packing_detail {
namespace {
void Add(std::size_t& bytes, std::size_t count, std::size_t width, std::size_t cap) {
    output::Require(width && bytes <= cap && count <= (cap-bytes)/width,
                    "Tied packing host byte cap exceeded");
    bytes += count*width;
}
}
std::size_t Preflight(const source::CanonicalData& source, const Data& d, PackingLimits limits) {
    const PackingLimits maximum;
    output::Require(limits.host_bytes && limits.host_bytes <= maximum.host_bytes &&
                    limits.masters && limits.masters <= maximum.masters &&
                    limits.nodes && limits.nodes <= maximum.nodes,
                    "Invalid tied packing limits");
    output::Require(!d.masters.empty() && d.masters.size() <= limits.masters &&
                    d.masters.size() == d.counts.masters &&
                    source.canonical_nodes <= limits.nodes &&
                    source.canonical_shells <= UINT32_MAX &&
                    d.slave_nodes.size() <= limits.nodes && d.master_nodes.size() <= limits.nodes,
                    "Tied packing source extent exceeds capacity");
    // Shared backing and the declaration payload are charged once. The two
    // decoded arrays and both final permutations coexist during preparation.
    // Decode also owns a temporary native-endian string alongside its vector.
    std::size_t bytes = sizeof(PackingData) + sizeof(source::CanonicalData) + 256;
    Add(bytes, d.owned_payload_bytes, 1, limits.host_bytes);
    for (const auto& array : source.arrays)
        Add(bytes, array.bytes.size(), 1, limits.host_bytes);
    Add(bytes, source.canonical_bytes.size(), 1, limits.host_bytes);
    Add(bytes, source.scope_bytes.size(), 1, limits.host_bytes);
    Add(bytes, source.parts.capacity(), sizeof(source::PartDeclaration), limits.host_bytes);
    Add(bytes, source.selected_parts.capacity(), sizeof(SourceId), limits.host_bytes);
    Add(bytes, source.excluded_parts.capacity(), sizeof(SourceId), limits.host_bytes);
    Add(bytes, source.canonical_shells, 12*sizeof(SourceId), limits.host_bytes);
    Add(bytes, source.canonical_nodes, 2*sizeof(SourceId), limits.host_bytes);
    Add(bytes, d.masters.size(), 2*sizeof(std::uint32_t), limits.host_bytes);
    return bytes;
}
} // namespace crash::modelio::tied_shell::packing_detail
