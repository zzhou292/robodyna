#include "VehicleConnectivity.h"
#include "ReportText.h"

namespace crash::cases::vehicle_startup::connectivity {
std::string ReportJson(const VehicleConnectivity& value) {
    const auto& physical = value.source().physical();
    const auto& canonical = physical.shell_source().references().source().canonical().data();
    detail::ReportIdentity identity{canonical.archive_sha256,canonical.inputs.source_member.sha256,
        canonical.inputs.canonical_manifest.sha256,canonical.inputs.tire_policy};
    if (const auto* joints = value.joint_model()) {
        identity.has_joints = true;
        identity.joint_source_instance = joints->model().source_instance_id();
        identity.joint_boundaries = joints->source().data().boundaries;
    }
    return detail::RenderReport(value.data(),physical.source_domain().domain().nodes(),value.forecast(),
                                identity,value.forecast().report_reservation);
}
} // namespace crash::cases::vehicle_startup::connectivity
