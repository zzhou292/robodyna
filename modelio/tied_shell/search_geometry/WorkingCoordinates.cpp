#include "Internal.h"
#include "modelio/source_assembly/NativeCoordinates.h"
namespace crash::modelio::tied_shell::search_detail {
void WorkingCoordinates(const source::CanonicalData& source, const Topology& topology,
        const std::string& member, SearchGeometryData& data, SearchGeometryLimits) {
    auto result = source_nodes::ReadNativeCoordinates(source, topology.nodes, data.canonical_nodes, member);
    data.working_positions = std::move(result.positions);
    data.source_roundtrip_changed_components = result.roundtrip_changed_components;
}
} // namespace crash::modelio::tied_shell::search_detail
