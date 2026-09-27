#include "Storage.h"
#include "Reports.h"
#include "SolidReadback.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InspectSolids(InitialInspection& out) {
    const auto& model = source.solids();
    std::vector<tl::fea::solids::Result18> a(model.solid18().size());
    std::vector<tl::fea::solids::Result6z> c(model.solid6z().size());
    std::vector<tl::fea::solids::Result18Law44> rear(model.solid18_law44().size());
    tl::fea::solids::BatchDiagnostics diagnostics;
    if(detail::HasSourceSolidControls(model)) {
        namespace s=tl::fea::solids;
        std::vector<s::ProfiledResult24> b(model.solid24().size());
        std::vector<s::ProfiledResult18Law90> foam(model.solid18_law90().size());
        detail::RequireSuccess(solids.CopyAcceptedResultsWithControls(out.stamp,
            {a.data(),a.size(),b.data(),b.size(),c.data(),c.size(),rear.data(),rear.size(),foam.data(),foam.size()},
            &diagnostics));
        for(const auto& parent:model.control_selection().parents()) {
            const auto expected=parent.source.icontrol?s::ResultProfile::NativeControlled:s::ResultProfile::Legacy;
            if(parent.family==s::Family::Solid24)
                output::Require(b.at(parent.family_index).history.profile()==expected,"HEPH readback control profile differs from source");
            if(parent.family==s::Family::Solid18Law90)
                output::Require(foam.at(parent.family_index).history.profile()==expected,"LAW90 readback control profile differs from source");
        }
        detail::CheckInitialSolidStamps(b);detail::CheckInitialSolidStamps(foam);
    } else {
        std::vector<tl::fea::solids::Result24> b(model.solid24().size());
        std::vector<tl::fea::solids::Result18Law90> foam(model.solid18_law90().size());
        detail::RequireSuccess(solids.CopyAcceptedResults(out.stamp,
            {a.data(),a.size(),b.data(),b.size(),c.data(),c.size(),rear.data(),rear.size(),foam.data(),foam.size()},
            &diagnostics));
        detail::CheckInitialSolidStamps(b);detail::CheckInitialSolidStamps(foam);
    }
    output::Require(diagnostics.valid && !diagnostics.has_completed_interval &&
        diagnostics.parent_count[0]==a.size() && diagnostics.parent_count[1]==model.solid24().size() &&
        diagnostics.parent_count[2]==c.size() && diagnostics.parent_count[3]==rear.size() &&
        diagnostics.parent_count[4]==model.solid18_law90().size(),"Initial typed solid cache shape/phase differs");
    detail::CheckInitialSolidStamps(a);detail::CheckInitialSolidStamps(c);detail::CheckInitialSolidStamps(rear);
    // Native TT0 caches may contain floating frame residuals. Do not replace
    // them with invented all-zero stress/force or perform a positive-dt step.
    out.solid_parents = a.size()+model.solid24().size()+c.size()+rear.size()+model.solid18_law90().size();
}
} // namespace crash::cases::vehicle_runtime
