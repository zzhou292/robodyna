#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::detail {
namespace hash = ::crash::cases::vehicle_self_contact::native::detail::digest;
std::string InputDigest(const coated::Inputs& in, const Packing& p, const Provenance& provenance, std::size_t cap) {
    hash::Fields digest("initial-surface-input-v1:" + provenance.source_digest + ":" +
        provenance.selection_digest + ":" + provenance.control_rule, cap);
    digest.Add<std::uint64_t>("physical_node_ids", in.nodes.size(), 1, [&](auto i) { return in.nodes[i].source_id; });
    digest.Add<double>("native_reader_coordinates", in.nodes.size(), 3, [&](auto i) {
        const auto x = in.nodes[i/3].native_position;
        return i%3 == 0 ? x.x : i%3 == 1 ? x.y : x.z;
    });
    digest.Add<std::uint64_t>("physical_shell_source", in.shells.size(), 2, [&](auto i) {
        const auto& row = in.shells[i/2];
        return i%2 ? row.part_id : row.primary.source_id;
    });
    digest.Add<std::uint32_t>("physical_shell_nodes", in.shells.size(), 4,
        [&](auto i) { return in.shells[i/4].primary.nodes[i%4]; });
    digest.Add<std::uint64_t>("physical_solid_source", in.solids.size(), 2, [&](auto i) {
        const auto& row = in.solids[i/2];
        return i%2 ? row.part_id : row.source_id;
    });
    digest.Add<std::uint32_t>("physical_reader_raw8", in.solids.size(), 8,
        [&](auto i) { return in.solids[i/8].nodes[i%8]; });
    digest.Add<std::uint64_t>("selected_parts", p.selected_parts.size(), 1,
        [&](auto i) { return p.selected_parts[i]; });
    const double units[]{in.units.length_m, in.units.mass_kg, in.units.time_s};
    digest.Add<double>("native_units", 1, 3, [&](auto i) { return units[i]; });
    return digest.Finish().sha256;
}
std::string OutputDigest(const std::vector<Face>& faces, const std::vector<std::uint8_t>& flags,
        const Certificate& certificate, const Provenance& provenance, std::size_t cap) {
    hash::Fields digest("initial-surface-output-v1:" + provenance.input_digest, cap);
    digest.Add<std::uint64_t>("typed_external_face_identity", faces.size(), 6, [&](auto i) {
        const auto& s = faces[i/6].source;
        const std::uint64_t fields[]{std::uint64_t(s.kind), s.element_id, s.part_id,
            s.canonical_row, s.source_line, s.solid_face};
        return fields[i%6];
    });
    digest.Add<std::uint32_t>("initial_ordered_nodes", faces.size(), 4,
        [&](auto i) { return faces[i/4].nodes[i%4]; });
    digest.Add<std::int32_t>("initial_raw_roles", faces.size(), 1,
        [&](auto i) { return std::int32_t(faces[i].raw_role); });
    digest.Add<std::uint32_t>("emitted_solid_observation", flags.size(), 1, [&](auto i) { return flags[i]; });
    const std::uint64_t proof[]{certificate.queried_solid_faces, certificate.matching_physical_shells,
        certificate.equal_node_key_groups, certificate.membership_complete, certificate.consumed_order_complete, certificate.differing_origin_groups};
    digest.Add<std::uint64_t>("order_certificate", 1, 6, [&](auto i) { return proof[i]; });
    return digest.Finish().sha256;
}
}
