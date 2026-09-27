#include "Internal.h"
#include "lib_src/collision/self_contact_filters/Environment.h"

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
Forecast Budget(const seed::PreCorrectionNodalSource& input, const ids::ImportMembers& members, Limits limits) {
    const Limits hard;
    if (!limits.host_bytes || limits.host_bytes > hard.host_bytes || !limits.nodes || limits.nodes > hard.nodes ||
        !limits.solids || limits.solids > hard.solids || !limits.parts || limits.parts > hard.parts ||
        !limits.source_blocks || limits.source_blocks > hard.source_blocks ||
        !limits.metadata_bytes || limits.metadata_bytes > hard.metadata_bytes)
        Reject(Status::ResourceLimit, "Invalid corrected nodal source capacity");
    if (!tlfea::contact::self_contact_filters::CompatibleHostArithmetic())
        Reject(Status::InvalidInput, "Corrected source requires RN, gradual underflow and masked floating traps");
    const auto& counts = input.counts();
    if (counts.nodes > limits.nodes || counts.solids > limits.solids)
        Reject(Status::ResourceLimit, "Corrected source model exceeds node/solid capacity");
    const auto& canonical = input.physical().shell_source().references().source().canonical();
    const auto imported = ids::ImportContext::Preflight(canonical, members);
    Forecast result;
    const auto add = [&](std::size_t& field, std::size_t count, std::size_t width) {
        if (!width || result.peak_bytes > limits.host_bytes || count > (limits.host_bytes-result.peak_bytes)/width)
            Reject(Status::ResourceLimit, "Complete corrected nodal source byte cap exceeded");
        const auto bytes = count*width;
        field += bytes;
        result.peak_bytes += bytes;
    };
    add(result.pre_correction_reservation, input.forecast().peak_bytes, 1);
    // Existing preflight is an inclusive upper bound; no private import layout
    // or retired parser RSS is reconstructed as an exact allocation claim.
    add(result.import_context_reservation, imported.total_bytes, 1);
    add(result.source_workspace, canonical.data().canonical_bytes.size(), 8);
    add(result.source_workspace, ids::Limits{}.member_bytes, 2);
    add(result.source_workspace, limits.source_blocks * ids::Limits{}.members, 768);
    add(result.source_workspace, limits.parts, 2*sizeof(Part)+2*sizeof(Section)+2*sizeof(PartControl)+512);
    add(result.source_workspace, limits.metadata_bytes, 4);
    add(result.source_workspace, 65536, 1);
    // New immutable effective rows/origins coexist with extraction maps and the
    // legacy public part-control copy. Imported/direct backing is shared.
    add(result.source_workspace, 2, modelio::solid_control::Limits{}.retained_bytes);
    add(result.correction_inputs, counts.solids, sizeof(c::Solid)+sizeof(std::uint64_t)+sizeof(unsigned char));
    add(result.correction_inputs, counts.nodes, sizeof(double));
    add(result.output_bytes, counts.nodes, sizeof(double));
    // Public numerical adapters enforce these upper bounds. Actual preflights
    // are checked against them after source rows exist; no copied arena formula.
    add(result.certificate_scratch, c::Limits{}.scratch_bytes, 1);
    add(result.correction_scratch, c::Limits{}.scratch_bytes, 1);
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
