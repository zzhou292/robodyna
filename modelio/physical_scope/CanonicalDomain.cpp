#include "CanonicalDomain.h"
#include "output/BoundedArrayIO.h"
#include "output/ArtifactIO.h"
#include <algorithm>

namespace crash::modelio::physical_scope {
namespace detail {
using output::Require;
void CheckDomain(const std::vector<std::uint64_t>& ids, const std::vector<double>& positions,
                 const std::vector<std::uint16_t>& roles, const tl::fea::NodalNodeDomain& domain,
                 std::uint64_t instance, std::size_t expected_count) {
    Require(ids.size() <= SIZE_MAX / 3 && positions.size() == ids.size() * 3 && roles.size() == ids.size() &&
        domain.prepared() && domain.source_instance_id() == instance && domain.node_count() >= expected_count,
        "Physical common-domain source or extent changed");
    Require(!ids.empty() && ids.front() &&
        std::adjacent_find(ids.begin(), ids.end(), std::greater_equal<std::uint64_t>()) == ids.end(),
        "Physical canonical source node order changed");
    std::size_t mapped = 0, declared = 0;
    for (std::size_t n = 0; n < ids.size(); ++n) {
        Require((roles[n] & ~127u) == 0, "Physical physical source role changed");
        const bool required = roles[n] & (PhysicalRoles | ProvisionalType25);
        const auto index = domain.Find(ids[n]);
        if (!required && index == SIZE_MAX) continue;
        Require(index < domain.node_count(), "Physical common domain omits an original physical node");
        const auto& actual = domain.nodes()[index];
        Require(actual.source_id == ids[n] && output::Bits(actual.position.x) == output::Bits(positions[3 * n]) &&
            output::Bits(actual.position.y) == output::Bits(positions[3 * n + 1]) &&
            output::Bits(actual.position.z) == output::Bits(positions[3 * n + 2]),
            "Physical common-domain coordinate differs from original canonical SI bits");
        mapped += required;
        ++declared;
    }
    Require(mapped == expected_count && declared == domain.node_count(),
            "Physical common domain contains a node outside authenticated original coordinates");
}
} // namespace detail
void ValidateDeclaredDomain(const PhysicalScope& source, const tl::fea::NodalNodeDomain& domain) {
    const auto& canonical = source.tied_source().canonical().data();
    const auto& id_array = source::FindArray(canonical, "node_ids");
    const auto& position_array = source::FindArray(canonical, "node_positions");
    const auto ids = output::arrays::Decode<std::uint64_t>(id_array.descriptor, id_array.bytes);
    const auto positions = output::arrays::Decode<double>(position_array.descriptor, position_array.bytes);
    detail::CheckDomain(ids, positions, source.data().node_roles, domain,
        source.point_mass_source().rigid_source().topology().source_instance_id(),
        source.data().counts.with_type25_nodes);
}
} // namespace crash::modelio::physical_scope
