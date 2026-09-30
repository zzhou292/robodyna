#include "Internal.h"
#include <iterator>
namespace crash::cases::vehicle_startup::tied_finalization_detail {
TiedFinalizationForecast Preflight(const tied::source::CanonicalData& canonical,
        const tied::Data& declaration, const tied::PackingData& packing,
        const tied::SearchGeometryData& geometry, const native_search::SearchDriverResult& result,
        TiedFinalizationLimits limits) {
    using output::Require;
    using tied_assessment_detail::Add;
    const TiedFinalizationLimits hard;
    const std::size_t values[] = {limits.host_bytes, limits.metadata_bytes, limits.source_files,
        limits.source_blocks, limits.native.max_nodes, limits.native.max_masters,
        limits.native.max_slaves, limits.native.max_host_bytes};
    const std::size_t maximum[] = {hard.host_bytes, hard.metadata_bytes, hard.source_files,
        hard.source_blocks, hard.native.max_nodes, hard.native.max_masters,
        hard.native.max_slaves, hard.native.max_host_bytes};
    for (std::size_t i = 0; i < std::size(values); ++i)
        Require(values[i] && values[i] <= maximum[i], "Invalid tied finalization limits");
    Require(canonical.canonical_bytes.size() <= limits.metadata_bytes &&
            !geometry.working_positions.empty() && geometry.working_positions.size() <= limits.native.max_nodes &&
            !geometry.masters.empty() && geometry.masters.size() <= limits.native.max_masters &&
            !declaration.slave_nodes.empty() && declaration.slave_nodes.size() <= limits.native.max_slaves &&
            result.rows.size() == declaration.slave_nodes.size() &&
            geometry.secondary_working_nodes.size() == result.rows.size() &&
            declaration.master_nodes.size() <= geometry.working_positions.size(),
            "Tied finalization count or metadata exceeds capacity");
    TiedFinalizationForecast out;
    out.retained_source_bytes = tied_assessment_detail::CanonicalPayload(canonical, limits.host_bytes);
    for (const auto bytes : {declaration.owned_payload_bytes, packing.owned_payload_bytes, geometry.owned_payload_bytes})
        Add(out.retained_source_bytes, bytes, 1, limits.host_bytes);
    Add(out.retained_source_bytes, sizeof(tied::source::CanonicalSource) +
        sizeof(tied::TiedShellDeclaration) + sizeof(tied::TiedShellPacking), 1, limits.host_bytes);
    // Same assessment backing: the authenticated source handles, result and
    // result rows are retained once. Its former driver/device scratch is gone.
    out.retained_assessment_bytes = sizeof(tied::TiedShellSearchGeometry) +
        sizeof(native_search::SearchDriverResult) + sizeof(TiedAssessmentForecast);
    Add(out.retained_assessment_bytes, result.rows.capacity(), sizeof(native_search::SearchDriverRow), limits.host_bytes);
    Add(out.input_staging_bytes, geometry.masters.size(), sizeof(std::array<std::uint32_t,4>), limits.host_bytes);
    Add(out.input_staging_bytes, declaration.master_nodes.size(), sizeof(std::uint32_t), limits.host_bytes);
    Add(out.input_staging_bytes, result.rows.size(), sizeof(native_search::SearchChoice), limits.host_bytes);
    // Existing source-reader DOM allowance; includes the small keyword census.
    Add(out.metadata_reservation_bytes, canonical.canonical_bytes.size(), 6, limits.host_bytes);
    Add(out.metadata_reservation_bytes, limits.source_files, 512, limits.host_bytes);
    out.finalizer_reservation_bytes = limits.native.max_host_bytes;
    out.fixed_bytes = sizeof(TiedSearchAssessment) + sizeof(native_search::FinalizedSearch) +
        sizeof(TiedFinalizationReceipt) + sizeof(TiedFinalizationForecast) + sizeof(Inputs);
    for (const auto bytes : {out.retained_source_bytes, out.retained_assessment_bytes,
            out.input_staging_bytes, out.metadata_reservation_bytes, out.finalizer_reservation_bytes, out.fixed_bytes})
        Add(out.total_host_bytes, bytes, 1, limits.host_bytes);
    return out;
}
}
