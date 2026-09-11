#include "Internal.h"
namespace crash::cases::vehicle_startup::tied_assessment_detail {
TiedAssessmentForecast Preflight(const tied::source::CanonicalData& canonical,
    const tied::Data& declaration, const tied::PackingData& packing,
    const tied::SearchGeometryData& geometry, TiedAssessmentLimits limits) {
    using output::Require;
    const auto& d = limits.driver;
    const native_search::SearchDriverLimits hard;
    Require(limits.host_bytes && limits.host_bytes <= TiedAssessmentLimits{}.host_bytes,
            "Invalid tied assessment host limit");
    Require(d.max_nodes && d.max_nodes <= hard.max_nodes && d.max_masters && d.max_masters <= hard.max_masters &&
            d.max_secondaries && d.max_secondaries <= hard.max_secondaries &&
            d.max_pairs && d.max_pairs <= hard.max_pairs && d.max_host_bytes && d.max_host_bytes <= hard.max_host_bytes &&
            d.max_device_bytes && d.max_device_bytes <= hard.max_device_bytes && d.axis <= 2,
            "Invalid tied assessment driver limits");
    Require(!geometry.working_positions.empty() && geometry.working_positions.size() <= d.max_nodes &&
            !geometry.masters.empty() && geometry.masters.size() <= d.max_masters &&
            !geometry.secondary_working_nodes.empty() && geometry.secondary_working_nodes.size() <= d.max_secondaries,
            "Tied assessment count exceeds capacity");
    TiedAssessmentForecast out;
    out.source_payload_bytes = CanonicalPayload(canonical, limits.host_bytes);
    Add(out.source_payload_bytes, declaration.owned_payload_bytes, 1, limits.host_bytes);
    Add(out.source_payload_bytes, packing.owned_payload_bytes, 1, limits.host_bytes);
    Add(out.source_payload_bytes, geometry.owned_payload_bytes, 1, limits.host_bytes);
    // Each backing owns one parent handle in addition to its declared data.
    Add(out.source_payload_bytes, sizeof(tied::source::CanonicalSource) +
        sizeof(tied::TiedShellDeclaration) + sizeof(tied::TiedShellPacking), 1, limits.host_bytes);
    Add(out.input_staging_bytes, geometry.working_positions.size(), sizeof(native_search::Vec3), limits.host_bytes);
    Add(out.input_staging_bytes, geometry.masters.size(), sizeof(native_search::SearchMasterInput), limits.host_bytes);
    out.fixed_bytes = sizeof(tied::TiedShellSearchGeometry) + sizeof(native_search::SearchDriverResult) +
        sizeof(TiedAssessmentForecast) + sizeof(Inputs);
    out.driver_host_reservation_bytes = d.max_host_bytes;
    out.device_limit_bytes = d.max_device_bytes;
    for (const auto part : {out.source_payload_bytes, out.input_staging_bytes,
                          out.fixed_bytes, out.driver_host_reservation_bytes})
        Add(out.total_host_bytes, part, 1, limits.host_bytes);
    return out;
}
} // namespace crash::cases::vehicle_startup::tied_assessment_detail
