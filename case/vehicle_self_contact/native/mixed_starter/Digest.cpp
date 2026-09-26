#include "Internal.h"
#include <cstring>
namespace crash::cases::vehicle_self_contact::native::mixed_starter::detail {
namespace hash = ::crash::cases::vehicle_self_contact::native::detail::digest;
namespace {
std::uint32_t FloatBits(float value) {
    std::uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}
}
std::string Digest(const Provenance& source, const s::Snapshot& value, const s::Input& input, std::size_t cap) {
    hash::Fields digest("mixed-starter-native-cache-v1:" + source.source_digest + ":" + source.post_gapm_digest, cap);
    digest.Add<std::uint64_t>("complete_node_ids", input.node_count, 1,
        [&](auto i) { return input.node_source_ids[i]; });
    digest.Add<std::uint64_t>("complete_working_position_bits", input.node_count, 3, [&](auto i) {
        const auto point = input.positions.at(std::uint32_t(i/3));
        return output::Bits(i%3 == 0 ? point.x : i%3 == 1 ? point.y : point.z);
    });
    digest.Add<std::uint64_t>("working_coordinate_context", 1, 4, [&](auto i) {
        const std::uint64_t row[]{std::uint64_t(input.coordinates), output::Bits(input.units.length_m),
            output::Bits(input.units.mass_kg), output::Bits(input.units.time_s)};
        return row[i];
    });
    digest.Add<std::uint64_t>("scope", 1, 7, [&](auto i) {
        const std::uint64_t row[]{value.source_generation, value.node_count, value.primary_count,
            value.shell_primary_count, value.main_count, value.raw_origin_count, value.starter.reference_count};
        return row[i];
    });
    digest.Add<std::uint64_t>("main_source_availability", value.main_count, 1,
        [&](auto i) { return value.mains[i].source_id; });
    digest.Add<std::int32_t>("main_global_role", value.main_count, 2,
        [&](auto i) { return i%2 ? value.mains[i/2].segment_type : value.mains[i/2].global_id; });
    digest.Add<std::uint32_t>("ordered_nodes", value.main_count, 4,
        [&](auto i) { return value.mains[i/4].nodes[i%4]; });
    digest.Add<std::int32_t>("neighbors_edges_references", value.main_count, 12, [&](auto i) {
        const auto& main = value.mains[i/12];
        const auto lane = i%12;
        return lane < 4 ? main.neighbors[lane] : lane < 8 ? main.neighbor_edges[lane-4] : main.normal_reference[lane-8];
    });
    digest.Add<std::uint32_t>("expanded_to_primary", value.main_count, 1,
        [&](auto i) { return value.expanded_to_primary[i]; });
    digest.Add<std::uint32_t>("optional_partner", value.primary_count, 1,
        [&](auto i) { return value.primary_to_partner[i]; });
    digest.Add<std::uint32_t>("primary_roles", value.primary_count, 1,
        [&](auto i) { return std::uint32_t(value.primary_roles[i]); });
    const auto identities = [&](const char* name, const s::PrimaryFaceIdentity* rows, std::size_t count) {
        digest.Add<std::uint64_t>(name, count, 5, [&](auto i) {
            const auto& id = rows[i/5];
            const std::uint64_t row[]{std::uint64_t(id.kind), id.physical_parent_id, id.local_face,
                std::uint64_t(id.origin), id.origin_count};
            return row[i%5];
        });
    };
    identities("primary_identities", value.primary_identities, value.primary_count);
    identities("complete_origins", value.raw_origins, value.raw_origin_count);
    digest.Add<std::uint32_t>("raw_to_primary", value.raw_origin_count, 1,
        [&](auto i) { return value.raw_origin_to_primary[i]; });
    const auto& post = *value.post_gapm;
    digest.Add<std::uint64_t>("post_gapm_phase", 1, 5, [&](auto i) {
        const std::uint64_t row[]{std::uint64_t(post.phase), post.source_generation,
            post.pre_shell_internal_count, std::uint64_t(post.incoming_solid_erosion), std::uint64_t(post.final_solid_erosion)};
        return row[i];
    });
    digest.Add<std::uint32_t>("primary_corner_permutation", value.primary_count, 4,
        [&](auto i) { return post.primary_corners[i/4].source_corner[i%4]; });
    digest.Add<std::uint64_t>("pre_shell_support", value.primary_count, 3, [&](auto i) {
        const auto& row = post.before_shell[i/3];
        const std::uint64_t words[]{row.first_solid_source_id, row.second_solid_source_id, row.unique_match_count};
        return words[i%3];
    });
    digest.Add<std::uint64_t>("final_physical_support", value.main_count, 3, [&](auto i) {
        const auto& row = post.final_support[i/3];
        const std::uint64_t words[]{std::uint64_t(row.first.kind), row.first.source_element_id, row.second_solid_source_id};
        return words[i%3];
    });
    digest.Add<std::uint32_t>("normal_offsets", value.starter.reference_count+1, 1,
        [&](auto i) { return value.normal_offsets[i]; });
    digest.Add<std::uint32_t>("normal_incidence", value.normal_incidence_count, 1,
        [&](auto i) { return value.normal_mains[i]; });
    digest.Add<std::uint32_t>("starter_normal_float_bits", value.main_count, 12, [&](auto i) {
        const auto& normal = value.starter.face_normals[i/3];
        return FloatBits(i%3 == 0 ? normal.x : i%3 == 1 ? normal.y : normal.z);
    });
    digest.Add<std::int32_t>("starter_boundary_boolean", value.starter.reference_count, 1,
        [&](auto i) { return value.starter.references[i].boundary; });
    digest.Add<std::uint32_t>("starter_bisector_float_bits", value.starter.reference_count, 6, [&](auto i) {
        const auto& normal = value.starter.references[i/6].bisector[i%6/3];
        return FloatBits(i%3 == 0 ? normal.x : i%3 == 1 ? normal.y : normal.z);
    });
    return digest.Finish().sha256;
}
}
