#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InspectConnections(InitialInspection& out) {
    {
        const auto count = execution.model().coefficients().type25()->connection_count();
        std::vector<tl::fea::type25::Evaluation> rows(count);
        tl::fea::type25::BatchDiagnostics diagnostics;
        detail::RequireSuccess(type25.CopyAcceptedResults(out.stamp,rows.data(),rows.size(),&diagnostics));
        output::Require(diagnostics.valid && !diagnostics.has_completed_interval && diagnostics.element_count==count,
                        "Initial TYPE25 cache shape/phase differs");
        out.type25_connections = count;
    }
    {
        const auto count = execution.model().beams().connection_count();
        std::vector<tl::fea::type13::Evaluation> rows(count);
        tl::fea::type13::BatchDiagnostics diagnostics;
        detail::RequireSuccess(type13.CopyAcceptedResults(out.stamp,rows.data(),rows.size(),&diagnostics));
        output::Require(diagnostics.valid && !diagnostics.has_completed_interval && diagnostics.element_count==count,
                        "Initial TYPE13 six-channel cache shape/phase differs");
        for (const auto& row:rows) {
            output::Require(row.native_history.active && !row.newly_failed,"Initial TYPE13 active role differs");
            for (const auto& channel:row.native_history.channels)
                output::Require(channel.accumulated_plastic_deformation==0 && channel.signed_work==0,
                                "Initial TYPE13 channel has advanced");
        }
        out.type13_connections = count;
    }
}
} // namespace crash::cases::vehicle_runtime
