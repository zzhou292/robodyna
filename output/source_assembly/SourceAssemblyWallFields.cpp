#include "SourceAssemblyWallFields.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace crash::output::assembly {
namespace wall_fields {
void CheckCase(const dynamics::SourceAssemblyWallCase& run) {
    Require(run.initialized()&&run.owner()&&run.bindings()&&run.setup()&&run.config()&&run.diagnostics(),
        "Assembly wall archive requires the initialized live case");
    Require(tl::fea::trial_identity::SameStamp(run.owner()->accepted(),run.diagnostics()->stamp),
        "Case observations do not identify its complete accepted owner stamp");
}
}
Document SourceAssemblyWallFrameFields(const dynamics::SourceAssemblyWallCase& run,const SourceAssemblyAcceptedOutput& output) {
    wall_fields::CheckCase(run);
    Require(output.nodal()&&output.nodal()->stamp(),"Assembly frame has no accepted output capture");
    Require(tl::fea::trial_identity::SameStamp(*output.nodal()->stamp(),run.owner()->accepted()),
        "Assembly output and live case identify different accepted states");
    return wall_fields::FrameDocument({output.mapping(),run.bindings(),run.setup(),output.nodal()->stamp(),output.nodal()->fields(),
        output.qeph(),output.t3(),output.diagnostics(),run.diagnostics(),run.accepted_contact(),run.config()->observe_force_stage,run.accepted_force_stage()});
}
Document SourceAssemblyWallConfiguration(const dynamics::SourceAssemblyWallCase& run,const SourceAssemblySurface& surface,const WallArchiveRequest& request) {
    wall_fields::CheckCase(run);Require(!run.owner()->accepted().epoch,"Assembly archive begins at the physical initial state only");
    return wall_fields::ConfigurationDocument(*run.bindings(),*run.setup(),*run.config(),surface,request);
}
std::string SourceAssemblyWallInterval(const tl::fea::NodalStamp& base,const dynamics::SourceAssemblyWallCase& run) {
    wall_fields::CheckCase(run);return wall_fields::IntervalRow(base,*run.diagnostics(),run.accepted_contact());
}
} // namespace crash::output::assembly
