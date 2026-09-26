#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::mixed_interface::detail {
namespace hash = ::crash::cases::vehicle_self_contact::native::detail::digest;
std::string Digest(const Provenance& provenance, const f::Snapshot& classified,
        const s::MixedSidesSnapshot& sides, const std::vector<RoleObservation>& roles,
        const Certificate& certificate, std::size_t cap) {
    hash::Fields digest("mixed-interface-before-support-v1:" + provenance.source_digest + ":" + provenance.initial_digest, cap);
    digest.Add<std::uint64_t>("scope_counts", 1, 4, [&](auto i) {
        const std::uint64_t values[]{sides.node_count, sides.primary_count, sides.shell_primary_count, sides.main_count};
        return values[i];
    });
    digest.Add<std::int32_t>("raw_IN24_roles", roles.size(), 1, [&](auto i) { return roles[i].native_role; });
    digest.Add<std::uint64_t>("unique_IN24_solid_source", roles.size(), 1,
        [&](auto i) { return roles[i].unique_coating_solid_eid; });
    digest.Add<std::uint32_t>("filtered_primary_nodes", classified.primary_count, 4,
        [&](auto i) { return classified.primary[i/4].nodes[i%4]; });
    const auto identities = [&](const char* name, const s::PrimaryFaceIdentity* rows, std::size_t count) {
        digest.Add<std::uint64_t>(name, count, 5, [&](auto i) {
            const auto& row = rows[i/5];
            const std::uint64_t fields[]{std::uint64_t(row.kind), row.physical_parent_id, row.local_face,
                std::uint64_t(row.origin), row.origin_count};
            return fields[i%5];
        });
    };
    identities("primary_identity_availability", sides.primary_identities, sides.primary_count);
    identities("all_raw_origins", sides.raw_origins, sides.raw_origin_count);
    digest.Add<std::uint32_t>("all_raw_to_primary", sides.raw_origin_count, 1,
        [&](auto i) { return sides.raw_origin_to_primary[i]; });
    digest.Add<std::uint32_t>("all_solid_flags", classified.physical_solid_count, 1,
        [&](auto i) { return classified.surface_solid_flags[i]; });
    digest.Add<std::uint64_t>("expanded_source_availability", sides.main_count, 1,
        [&](auto i) { return sides.mains[i].source_id; });
    digest.Add<std::uint32_t>("expanded_ordered_nodes", sides.main_count, 4,
        [&](auto i) { return sides.mains[i/4].nodes[i%4]; });
    digest.Add<std::int32_t>("expanded_global_id_role", sides.main_count, 2,
        [&](auto i) { return i%2 ? sides.mains[i/2].segment_type : sides.mains[i/2].global_id; });
    digest.Add<std::uint32_t>("expanded_to_primary", sides.main_count, 1,
        [&](auto i) { return sides.expanded_to_primary[i]; });
    digest.Add<std::uint32_t>("optional_partner", sides.primary_count, 1,
        [&](auto i) { return sides.primary_to_partner[i]; });
    const std::uint64_t proof[]{certificate.raw_shells, certificate.raw_solids, certificate.unique_coatings,
        certificate.ordinary_shells, certificate.filtered_primaries, certificate.shell_primaries,
        certificate.solid_primaries, certificate.multi_origin_primaries, certificate.coalesced_origins,
        certificate.unique_selected_membership, certificate.complete_origins, certificate.complete_solid_flags};
    digest.Add<std::uint64_t>("source_certificate", 1, std::size(proof), [&](auto i) { return proof[i]; });
    return digest.Finish().sha256;
}
}
