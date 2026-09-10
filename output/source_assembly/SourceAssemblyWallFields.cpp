#include "SourceAssemblyWallFields.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace crash::output::assembly {
namespace wall_fields {
void CheckCase(const dynamics::SourceAssemblyWallCase& run) {
    Require(run.initialized()&&run.owner()&&run.bindings()&&run.setup()&&run.config()&&run.diagnostics(),
        "Assembly wall archive requires the initialized live case");
    CheckWallSourceSchema(run.bindings()->source().data().schema);
    Require(tl::fea::trial_identity::SameStamp(run.owner()->accepted(),run.diagnostics()->stamp),
        "Case observations do not identify its complete accepted owner stamp");
}
}
wall_fields::FrameView wall_fields::AcceptedFrameView(const dynamics::SourceAssemblyWallCase& run,
        const SourceAssemblyAcceptedOutput& output) {
    CheckCase(run);
    Require(output.nodal()&&output.nodal()->stamp(),"Assembly frame has no accepted output capture");
    Require(tl::fea::trial_identity::SameStamp(*output.nodal()->stamp(),run.owner()->accepted()),
        "Assembly output and live case identify different accepted states");
    FrameView view{output.mapping(),run.bindings(),run.setup(),output.nodal()->stamp(),output.nodal()->fields(),
        output.qeph(),output.t3(),output.diagnostics(),run.diagnostics(),run.accepted_contact(),
        run.config()->observe_force_stage,run.accepted_force_stage(),run.accepted_connectors()};
    CheckFrame(view);
    if(view.stamp->epoch)CheckContactPhase(view);
    return view;
}
Document SourceAssemblyWallFrameFields(const dynamics::SourceAssemblyWallCase& run,const SourceAssemblyAcceptedOutput& output) {
    return wall_fields::FrameDocument(wall_fields::AcceptedFrameView(run,output));
}
Document SourceAssemblyWallConfiguration(const dynamics::SourceAssemblyWallCase& run,const SourceAssemblySurface& surface,const WallArchiveRequest& request) {
    wall_fields::CheckCase(run);Require(!run.owner()->accepted().epoch,"Assembly archive begins at the physical initial state only");
    return wall_fields::ConfigurationDocument(*run.bindings(),*run.setup(),*run.config(),surface,request);
}
std::string SourceAssemblyWallInterval(const tl::fea::NodalStamp& base,const dynamics::SourceAssemblyWallCase& run) {
    return interval::CsvRow(SourceAssemblyWallIntervalValues(base,run));
}
interval::Values SourceAssemblyWallIntervalValues(const tl::fea::NodalStamp& base,const dynamics::SourceAssemblyWallCase& run) {
    wall_fields::CheckCase(run);
    return wall_fields::IntervalValues(base,*run.diagnostics(),run.accepted_contact());
}
} // namespace crash::output::assembly
