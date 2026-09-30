#include "Internal.h"
#include "../TopologyDigestFields.h"
namespace crash::cases::vehicle_self_contact::native::coated::detail {
Digest InputDigest(const Inputs& input, const Classification& classified, const Order* order,
        Config config, const std::string& source_binding, std::size_t cap) {
    output::arrays::CheckHash(source_binding);
    output::Require(classified.roles.size() == input.shells.size(), "Incomplete coating source role digest");
    native::detail::digest::Fields fields("v5-declared-reader-coating-v1:" + source_binding, cap);
    const std::uint32_t policies[]{std::uint32_t(config.scope), std::uint32_t(config.coordinates),
        std::uint32_t(config.order), std::uint32_t(config.membership)};
    fields.Add<std::uint32_t>("policy", 1, 4, [&](auto i) { return policies[i]; });
    fields.Add<std::uint32_t>("role_readiness", 1, 2,
        [&](auto i) { return std::uint32_t(i == 0 ? classified.complete : classified.contact_complete); });
    fields.Add<double>("units", 1, 3, [&](auto i) { return i == 0 ? input.units.length_m : i == 1 ? input.units.mass_kg : input.units.time_s; });
    fields.Add<std::uint64_t>("physical_node_nid", input.nodes.size(), 1, [&](auto i) { return input.nodes[i].source_id; });
    fields.Add<std::uint32_t>("canonical_node_row", input.nodes.size(), 1, [&](auto i) { return input.nodes[i].canonical_row; });
    fields.Add<double>("original_native_position", input.nodes.size(), 3, [&](auto i) {
        const auto x = input.nodes[i/3].native_position; return i%3 == 0 ? x.x : i%3 == 1 ? x.y : x.z;
    });
    fields.Add<std::uint64_t>("physical_shell_eid_pid", input.shells.size(), 2, [&](auto i) {
        const auto& x = input.shells[i/2]; return i%2 == 0 ? x.primary.source_id : x.part_id;
    });
    fields.Add<std::uint32_t>("physical_shell_source", input.shells.size(), 5, [&](auto i) {
        const auto& x = input.shells[i/5];
        const std::uint32_t values[]{x.canonical_row, x.source_line, x.physical_parent,
            std::uint32_t(x.primary.layout), std::uint32_t(x.contact_selected)}; return values[i%5];
    });
    fields.Add<std::uint32_t>("physical_shell_nodes", input.shells.size(), 4,
        [&](auto i) { return input.shells[i/4].primary.nodes[i%4]; });
    fields.Add<std::uint64_t>("physical_solid_eid_pid", input.solids.size(), 2, [&](auto i) {
        const auto& x = input.solids[i/2]; return i%2 == 0 ? x.source_id : x.part_id;
    });
    fields.Add<std::uint32_t>("physical_solid_source", input.solids.size(), 6, [&](auto i) {
        const auto& x = input.solids[i/6];
        const std::uint32_t values[]{x.canonical_row, x.source_line, x.family, x.reference_index,
            std::uint32_t(x.kind), std::uint32_t(x.phase)}; return values[i%6];
    });
    fields.Add<std::uint32_t>("reader_before_initia_raw8", input.solids.size(), 8,
        [&](auto i) { return input.solids[i/8].nodes[i%8]; });
    fields.Add<std::uint32_t>("resolved_role_and_membership", classified.roles.size(), 3, [&](auto i) {
        const auto& x = classified.roles[i/3]; const std::uint32_t values[]{std::uint32_t(x.state), x.matches, x.first_solid};
        return values[i%3];
    });
    fields.Add<double>("unique_match_orientation_determinant", classified.roles.size(), 1,
        [&](auto i) { return classified.roles[i].determinant; });
    fields.Add<std::uint32_t>("primary_to_physical", order ? order->primary_to_physical.size() : 0, 1,
        [&](auto i) { return order->primary_to_physical[i]; });
    fields.Add<std::uint32_t>("physical_to_primary", order ? order->physical_to_primary.size() : 0, 1,
        [&](auto i) { return order->physical_to_primary[i]; });
    return fields.Finish();
}
Digest OutputDigest(const s::Snapshot& snapshot, const std::string& input, std::size_t cap) {
    return native::detail::TopologyDigest(snapshot, input, cap);
}
} // namespace crash::cases::vehicle_self_contact::native::coated::detail
