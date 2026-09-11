#include "VehicleConnectivity.h"
#include "ReportText.h"

namespace crash::cases::vehicle_startup::connectivity {
std::string ReportJson(const VehicleConnectivity& value) {
    const auto& physical = value.source().physical();
    const auto& canonical = physical.shell_source().references().source().canonical().data();
    const detail::ReportIdentity identity{canonical.archive_sha256,canonical.inputs.source_member.sha256,
        canonical.inputs.canonical_manifest.sha256,canonical.inputs.tire_policy};
    return detail::RenderReport(value.data(),physical.source_domain().domain().nodes(),value.forecast(),
                                identity,value.forecast().report_reservation);
}
} // namespace crash::cases::vehicle_startup::connectivity
