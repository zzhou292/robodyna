#include "Declaration.h"
namespace crash::cases::vehicle_native_contact::activity {
std::array<native::Forecast,2> PreflightPlans(const detail::SourceInputs& in,const Declaration& declaration,
    const tlfea::contact::radioss_type25::MixedMovingMainSource& self,
    const tlfea::contact::radioss_type25::FixedMainSource& wall,native::Limits limits) {
    output::Require(declaration.joints().SharesStorage(in.owner.joints()),"Activity declaration belongs to another actual joint model");
    const native::PhysicalSources physical{in.owner.physical(),&declaration.joints()};
    std::array<native::Forecast,2> out{{native::Plan::Preflight(physical,self.starter,declaration.self(),limits),
        native::Plan::Preflight(physical,wall,declaration.wall(),limits)}};
    for(const auto& value:out) {
        output::Require(value.report.status==tlfea::contact::radioss_type25::TransactionStatus::Ok,value.report.message);
        output::Require(value.counts.families==declaration.coverage().families&&value.counts.nodes==declaration.coverage().physical_nodes,
            "SourcePlan census differs from authenticated executed-model support");
    }
    return out;
}
}
