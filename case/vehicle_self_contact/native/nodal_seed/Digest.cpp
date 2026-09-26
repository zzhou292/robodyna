#include "Internal.h"
#include "../TopologyDigestFields.h"

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
std::string ContributorDigest(const Packed& input, const Provenance& provenance, std::size_t cap) {
    // Reuse the bounded, typed, padding-free field/chunk digest implementation.
    // The binding string gives this distinct pre-correction product its scope.
    const std::string binding = "robo_dyna.pre_correction_contact_seed.v1:" +
        provenance.canonical_sha256 + ":" + provenance.import_source_digest + ":" + provenance.spring_mapping_digest;
    ::crash::cases::vehicle_self_contact::native::detail::digest::Fields fields(binding, cap);
    fields.Add<double>("working_units", 1, 3, [&](auto i) {
        return i == 0 ? provenance.units.length_m : i == 1 ? provenance.units.mass_kg : provenance.units.time_s;
    });
    fields.Add<std::uint64_t>("contributor_identity", input.contributors.size(), 8, [&](auto index) {
        const auto& row = input.contributors[index / 8];
        switch (index % 8) {
        case 0: return std::uint64_t(row.kind);
        case 1: return std::uint64_t(row.channel);
        case 2: return row.original_id;
        case 3: return row.native_id;
        case 4: return row.part_id;
        case 5: return row.material_id;
        case 6: return std::uint64_t(row.source_index);
        default: return std::uint64_t(row.slots);
        }
    });
    fields.Add<std::uint32_t>("contributor_nodes", input.contributors.size(), 8,
        [&](auto i) { return input.contributors[i / 8].nodes[i % 8]; });
    fields.Add<std::uint32_t>("volume_nodes", input.volumes.size(), 1,
        [&](auto i) { return input.volumes[i].node; });
    fields.Add<double>("volume_operands", input.volumes.size(), 2, [&](auto i) {
        return i % 2 ? input.volumes[i / 2].bulk_volume : input.volumes[i / 2].volume;
    });
    fields.Add<std::uint32_t>("direct_nodes", input.stiffness.size(), 1,
        [&](auto i) { return input.stiffness[i].node; });
    fields.Add<double>("direct_stiffness", input.stiffness.size(), 1,
        [&](auto i) { return input.stiffness[i].stiffness; });
    fields.Add<double>("physical_shell_young_thickness", input.shells.size(), 2, [&](auto i) {
        return i % 2 ? input.shells[i / 2].structural_thickness : input.shells[i / 2].young;
    });
    return fields.Finish().sha256;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
