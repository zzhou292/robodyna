#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InspectStructuralBeams(InitialInspection& output) {
    if(!beam18) return;
    const auto* model=execution.model().structural_beams();
    std::vector<tl::fea::beam18::Result> rows(model->parents().size());
    output::Require(rows.capacity()<=forecast.readback_temporary_bytes/sizeof(rows[0]),
        "Structural beam readback allocation exceeds the complete forecast");
    tl::fea::beam18::BatchDiagnostics diagnostics;
    detail::RequireSuccess(beam18->CopyAcceptedResults(owner.accepted(),{rows.data(),rows.size()},&diagnostics));
    output::Require(diagnostics.valid && diagnostics.phase==tl::fea::beam18::BatchPhase::Accepted &&
        diagnostics.epoch==0 && diagnostics.time==0 && diagnostics.parent_count==rows.size() &&
        !diagnostics.has_completed_interval,"Initial structural beam diagnostics claim an interval");
    for(const auto& row:rows) {
        output::Require(row.stamp.sample_index==0 && row.stamp.time_s==0 &&
            row.history.filtered_neutral_rate_per_s==0 && row.history.plastic_work_j==0 &&
            row.diagnostics.translation_stiffness_n_m>0 && row.diagnostics.rotation_stiffness_nm>0,
            "Initial structural beam history or native stiffness differs from its virgin cache");
        for(const auto& point:row.history.point)
            output::Require(point.plastic_strain==0,"Initial structural beam point is not virgin");
    }
    output.structural_beams=rows.size();
}
} // namespace crash::cases::vehicle_runtime
