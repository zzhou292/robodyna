#include "Internal.h"
#include "../TopologyDigestFields.h"
namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
namespace {
std::string Digest(const std::vector<PartControl>& parts, const InterfaceCensus& interfaces,
    const std::string& source_digest, std::size_t cap) {
    ::crash::cases::vehicle_self_contact::native::detail::digest::Fields fields(
        "native-effective-control-single-mid-v1:" + source_digest, cap);
    fields.Add<std::uint64_t>("parts", parts.size(), 6, [&](auto index) {
        const auto& part = parts[index / 6];
        switch (index % 6) {
        case 0: return part.part_id;
        case 1: return part.section_id;
        case 2: return part.material_id;
        case 3: return part.native_property_id;
        case 4: return std::uint64_t(part.directly_requested);
        default: return std::uint64_t(part.effective_control);
        }
    });
    const std::uint64_t census[]{std::uint64_t(interfaces.disposition), interfaces.type25_sources,
        interfaces.type2_sources, interfaces.interior_sources, interfaces.rigid_wall_sources,
        interfaces.checked_source_blocks};
    fields.Add<std::uint64_t>("complete_interface_source_profile", 1, 6, [&](auto i) { return census[i]; });
    return fields.Finish().sha256;
}
}
Context ReadContext(const seed::PreCorrectionNodalSource& input, const ids::ImportMembers& members,
                    const ids::ImportContext& imported, Limits limits) {
    if (imported.data().diagnostic.status != ids::Readiness::Ready ||
        imported.data().source_digest != input.provenance().import_source_digest)
        Reject(Status::InvalidInput, "Correction and pre-correction do not share the same closed import context");
    const auto prepared = controls::EffectiveSource::Prepare(input.solid_control_declarations(), imported, members,
        {limits.parts, limits.source_blocks, limits.metadata_bytes, controls::Limits{}.retained_bytes});
    if (!prepared.source) RejectControl(prepared.report);
    Context result;
    result.source = prepared.source;
    result.parts = prepared.source->data().parts;
    result.interfaces = prepared.source->data().interfaces;
    result.source_digest = prepared.source->data().source_digest;
    // Preserve this legacy contact admission here, not in structural source semantics.
    if (!result.interfaces.type25_sources || !result.interfaces.interior_sources)
        Reject(Status::UnsupportedSource, "Missing admitted original interface/control source");
    result.property_digest = Digest(result.parts, result.interfaces, result.source_digest + ":" +
        input.provenance().contributor_digest, limits.metadata_bytes);
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
