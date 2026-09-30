#include "Internal.h"
#include "../TopologyDigestFields.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
std::string Digest(const Provenance& p, const Certificate& c, const std::vector<PrimaryBinding>& bindings,
    const std::vector<CandidateOwner>& owners, const std::vector<double>& coefficients, std::size_t cap) {
    for (const auto* hash : {&p.input_digest, &p.topology_digest, &p.property_digest, &p.material_digest, &p.import_digest})
        output::arrays::CheckHash(*hash);
    native::detail::digest::Fields fields("v5-selected-shell-main-coefficients-v1:" + p.input_digest +
        p.topology_digest + p.property_digest + p.material_digest + p.import_digest, cap);
    fields.Add<std::uint32_t>("source_phase", 1, 2, [&](auto i) {
        // Phase1: post shell head/tails and BUILD_CNEL, pre INITIA main K.
        return i == 0 ? std::uint32_t(p.coordinates) : 1u;
    });
    fields.Add<double>("native_units", 1, 3, [&](auto i) {
        return i == 0 ? p.units.length_m : i == 1 ? p.units.mass_kg : p.units.time_s;
    });
    const std::uint64_t counts[]{c.primaries, c.coated, c.negative_support_volumes, c.unique_owners,
        c.corner_owners, c.material_group_owners, c.unresolved_owners, c.physical_duplicate_groups,
        c.physical_duplicate_rows, std::uint64_t(c.orientation_identity), std::uint64_t(c.owners_complete), std::uint64_t(c.grouping_controls_certified)};
    fields.Add<std::uint64_t>("source_certificate", 1, 12, [&](auto i) { return counts[i]; });
    fields.Add<std::uint64_t>("primary_source_bindings", bindings.size(), 13, [&](auto i) {
        const auto& b = bindings[i/13];
        const std::uint64_t values[]{b.contact_element, b.contact_part, b.support_element, b.support_part,
            b.solid_element, b.solid_part, b.contact_physical, b.support_physical, b.winner_begin,
            b.winner_count, b.partner, std::uint64_t(b.role), std::uint64_t(b.owner)};
        return values[i%13];
    });
    fields.Add<std::uint64_t>("all_equal_DX_ST_candidate_owners", owners.size(), 3, [&](auto i) {
        const auto& owner = owners[i/3];
        return i%3 == 0 ? owner.element : i%3 == 1 ? owner.part : owner.physical;
    });
    fields.Add<double>("expanded_main_native_K", coefficients.size(), 1, [&](auto i) { return coefficients[i]; });
    return fields.Finish().sha256;
}
}
