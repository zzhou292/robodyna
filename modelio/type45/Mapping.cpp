#include "Internal.h"
#include <algorithm>

namespace crash::modelio::type45 {
native::GeometryInput Row::Geometry() const noexcept {
    native::GeometryInput result;
    result.source_joint_id = source_id;
    const unsigned count = property_index == 0 ? 2 : 3;
    for (unsigned n = 0; n < count; ++n) {
        result.source_node_id[n] = nodes[n].source_id;
        result.position_m[n] = nodes[n].position_m;
    }
    return result;
}
namespace detail {
std::vector<BodyMember> Members(const physical_domain::VehiclePhysicalDomain& source) {
    std::vector<BodyMember> result;
    result.reserve(32768);
    const auto& scope = source.source().data();
    for (const auto& selection : source.plain_groups()) {
        Require(selection.source_group < scope.plain_groups.size(), "Joint plain-group source association changed");
        const auto& group = scope.plain_groups[selection.source_group];
        for (const auto& member : group.members) {
            const bool retained = std::find(selection.members.begin(), selection.members.end(), member.node) != selection.members.end();
            Require(result.size() < 32768, "Joint source memberships exceed cap");
            result.push_back({member.node, {BodyKind::PlainGroup, group.id, selection.source_group, retained}});
        }
    }
    for (std::size_t root = 0; root < scope.part_roots.size(); ++root) {
        const auto& group = scope.part_roots[root];
        for (const auto& member : group.members) {
            Require(result.size() < 32768, "Joint source memberships exceed cap");
            result.push_back({member.node, {BodyKind::PartRoot, group.id, root, true}});
        }
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.node < b.node; });
    for (std::size_t i = 1; i < result.size(); ++i)
        Require(result[i - 1].node != result[i].node, "Ambiguous original joint body membership");
    return result;
}
void Map(std::vector<Row>& rows, const std::vector<std::uint64_t>& ids, const std::vector<double>& xyz,
         const tl::fea::NodalNodeDomain& domain, const std::vector<BodyMember>& members) {
    Require(domain.prepared() && ids.size() <= 524288 && xyz.size() == ids.size() * 3 && rows.size() <= 64,
            "Joint coordinate/domain extents changed");
    Require(std::is_sorted(ids.begin(), ids.end()) && std::adjacent_find(ids.begin(), ids.end()) == ids.end(),
            "Joint canonical node identities are not unique/ordered");
    for (std::size_t i = 1; i < members.size(); ++i)
        Require(members[i - 1].node < members[i].node, "Joint membership order/uniqueness changed");
    auto staged = rows;
    for (auto& row : staged) {
        Require(row.source_node_count <= row.nodes.size(), "Joint original node count changed");
        for (unsigned slot = 0; slot < row.source_node_count; ++slot) {
            auto& node = row.nodes[slot];
            const auto found = std::lower_bound(ids.begin(), ids.end(), node.source_id);
            Require(found != ids.end() && *found == node.source_id, "Original joint node is absent from canonical geometry");
            node.canonical_index = static_cast<std::size_t>(found - ids.begin());
            const auto n = node.canonical_index;
            node.position_m = {xyz[3*n], xyz[3*n+1], xyz[3*n+2]};
            Require(std::isfinite(node.position_m.x) && std::isfinite(node.position_m.y) && std::isfinite(node.position_m.z),
                    "Nonfinite original joint coordinate");
            node.domain_index = domain.Find(node.source_id);
            if (node.domain_index != SIZE_MAX) {
                const auto position = domain.nodes()[node.domain_index].position;
                Require(output::Bits(position.x) == output::Bits(node.position_m.x) &&
                    output::Bits(position.y) == output::Bits(node.position_m.y) &&
                    output::Bits(position.z) == output::Bits(node.position_m.z), "Joint domain coordinate bits changed");
            }
            node.body = {};
            const auto body = std::lower_bound(members.begin(), members.end(), node.source_id,
                [](const BodyMember& value, std::uint64_t id) { return value.node < id; });
            if (body != members.end() && body->node == node.source_id) {
                node.body = body->body;
                Require(node.body.source_id && node.body.kind != BodyKind::None &&
                    node.body.retained == (node.domain_index != SIZE_MAX), "Joint body/domain disposition differs");
            }
        }
    }
    rows.swap(staged);
}
} // namespace detail
} // namespace crash::modelio::type45
