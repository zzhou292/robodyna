#include "Internal.h"
#include "../TopologyDigestFields.h"

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
std::string MaterialDigest(const std::vector<c::Solid>& rows, const Context& context, std::size_t cap) {
    ::crash::cases::vehicle_self_contact::native::detail::digest::Fields fields(
        "native-post-updmat-control-rows-v1:" + context.property_digest, cap);
    fields.Add<std::uint32_t>("post_initia_raw_nodes", rows.size(), 8,
        [&](auto i) { return rows[i/8].nodes[i%8]; });
    fields.Add<std::int32_t>("effective_control", rows.size(), 1,
        [&](auto i) { return std::int32_t(rows[i].control); });
    fields.Add<double>("native_pm32_pm107", rows.size(), 2,
        [&](auto i) { return i%2 ? rows[i/2].controlled_bulk : rows[i/2].bulk; });
    return fields.Finish().sha256;
}
std::string CertificateDigest(const c::OrderCertificate& certificate,
    const std::string& material, std::size_t cap) {
    ::crash::cases::vehicle_self_contact::native::detail::digest::Fields fields(
        "certified-bit-identical-factors-per-node-v1:" + material, cap);
    const std::uint64_t values[]{certificate.node_count, certificate.solid_count,
        certificate.controlled_solids, certificate.affected_nodes};
    fields.Add<std::uint64_t>("complete_equivalence_certificate", 1, 4, [&](auto i) { return values[i]; });
    return fields.Finish().sha256;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
