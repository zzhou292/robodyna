#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail {
namespace {
void Add(std::size_t& bytes, std::size_t count, std::size_t width = 1) {
    if (!width || count>(SIZE_MAX-bytes)/width)Reject(Status::ResourceLimit,"Main source reservation overflow");
    bytes+=count*width;
}
}
Forecast Budget(const c::CorrectedNodalSource& corrected, const modelio::self_contact::OriginalSelection& selection, Limits limits) {
    const Limits hard;
    if (!limits.host_bytes || limits.host_bytes>hard.host_bytes || !limits.parts || limits.parts>hard.parts ||
        !limits.metadata_bytes || limits.metadata_bytes>hard.metadata_bytes)
        Reject(Status::ResourceLimit,"Invalid main source capacity limits");
    const auto& physical = corrected.pre_correction().physical();
    Forecast result; result.coating = coated::Preflight(physical, selection, coated::ConfigFor(physical), limits.coating);
    if (!result.coating.admitted)Reject(Status::ResourceLimit,"Complete main geometry source reservation rejected");
    result.shared_corrected_reservation = corrected.forecast().peak_bytes;
    const auto h = physical.shell_source().references().rows().size();
    const auto p = selection.data().counts.retained_shells;
    const auto g = result.coating.topology.expanded_mains;
    if (corrected.part_controls().size()>limits.parts || result.coating.retained_model_reservation>result.shared_corrected_reservation)
        Reject(Status::ResourceLimit,"Main source shared backing/part reservation differs");
    Add(result.part_operands, 2*limits.parts, sizeof(PartValue)+256);
    Add(result.part_operands, 2*h, sizeof(ShellValue));
    Add(result.face_keys, 2*h, sizeof(FaceKey));
    // Full possible-winner inventory cannot exceed physical shells per primary.
    // One physical node-set group can back several selected duplicate primaries
    // only if old topology admits them; it does not. Thus total candidates<=h.
    Add(result.output_bindings, 2*p, sizeof(PrimaryBinding));
    Add(result.output_bindings, 2*h, sizeof(CandidateOwner));
    Add(result.output_bindings, 4*h, sizeof(std::size_t)); // transient winner/order comparisons
    Add(result.output_coefficients, 2*g, sizeof(double));
    Add(result.source_metadata, 16, limits.metadata_bytes);
    Add(result.source_metadata, 2, modelio::self_contact::Limits{}.combine_member_bytes);
    Add(result.source_metadata, sizeof(Provenance) + sizeof(Certificate) + sizeof(Forecast) + 4096);
    for (const auto* text : {&corrected.provenance().property_digest, &corrected.provenance().material_digest,
            &corrected.provenance().source_digest})
        Add(result.source_metadata, 2, text->capacity() + 1);
    Add(result.source_metadata, 3 * 2, 65); // New bounded SHA256 strings, independent of metadata cap.
    Add(result.source_metadata, 16, selection.canonical().data().canonical_bytes.size()); // Two JSON DOM phases.

    result.peak_bytes = result.coating.peak_bytes-result.coating.retained_model_reservation;
    Add(result.peak_bytes, result.shared_corrected_reservation);
    Add(result.peak_bytes, result.part_operands); Add(result.peak_bytes, result.face_keys);
    Add(result.peak_bytes, result.output_bindings); Add(result.peak_bytes, result.output_coefficients);
    Add(result.peak_bytes, result.source_metadata);
    if (result.peak_bytes>limits.host_bytes)Reject(Status::ResourceLimit,"Complete selected-shell main source exceeds host envelope");
    CheckContext(corrected, selection);
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::main_coefficients::detail
