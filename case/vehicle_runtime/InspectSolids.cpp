#include "Storage.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InspectSolids(InitialInspection& out) {
    const auto& model = execution.model().solids();
    std::vector<tl::fea::solids::Result18> a(model.solid18().size());
    std::vector<tl::fea::solids::Result24> b(model.solid24().size());
    std::vector<tl::fea::solids::Result6z> c(model.solid6z().size());
    tl::fea::solids::BatchDiagnostics diagnostics;
    detail::RequireSuccess(solids.CopyAcceptedResults(out.stamp,
        {a.data(),a.size(),b.data(),b.size(),c.data(),c.size()},&diagnostics));
    output::Require(diagnostics.valid && !diagnostics.has_completed_interval &&
        diagnostics.parent_count[0]==a.size() && diagnostics.parent_count[1]==b.size() &&
        diagnostics.parent_count[2]==c.size(),"Initial typed solid cache shape/phase differs");
    const auto initial = [](const auto& rows) {
        for (const auto& row:rows) output::Require(row.stamp.sample_index==0 && row.stamp.time_s==0,
                                                  "Solid constructor cache has advanced");
    };
    initial(a);
    initial(b);
    initial(c);
    // Native TT0 caches may contain floating frame residuals. Do not replace
    // them with invented all-zero stress/force or perform a positive-dt step.
    out.solid_parents = a.size()+b.size()+c.size();
}
} // namespace crash::cases::vehicle_runtime
