#include "Internal.h"
#include <cmath>

namespace crash::modelio::tied_shell::search_detail {
SearchGeometryData Build(const source::CanonicalData& source, const Data& declaration,
        const PackingData& packing, const std::string& member, SearchGeometryLimits limits) {
    const auto forecast = Preflight(source, declaration, packing, limits);
    Require(member.size() == source.inputs.source_member.bytes &&
            output::Sha256(member) == source.inputs.source_member.sha256,
            "Tied search original member identity changed");
    source::CheckUnits(source.inputs.units);
    Require(source.inputs.units.length == "mm" && source.inputs.units.length_to_m == .001,
            "Tied search requires original millimetre coordinates");
    SearchGeometryData staged;
    staged.startup_budget_bytes = forecast;
    staged.working_length_to_m = source.inputs.units.length_to_m;
    const auto topology = ReadTopology(source, limits);
    Associate(topology, declaration, packing, staged, limits);
    Properties(source, declaration, topology, member, staged, limits);
    WorkingCoordinates(source, topology, member, staged, limits);
    Resolve(staged);
    staged.owned_payload_bytes = OwnedPayload(staged, limits);
    return staged;
}
} // namespace crash::modelio::tied_shell
