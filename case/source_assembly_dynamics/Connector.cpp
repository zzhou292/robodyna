#include "State.h"
#include <cmath>

namespace crash::cases::source_assembly_dynamics {
namespace {
Report ConnectorReport(const fe::type25::BatchReport& r,const source_assembly::SourceAssemblyBindings& b) noexcept {
    const auto* model=b.connectors();
    return BatchReport(r,model&&r.element<model->connection_count()?model->connections()[r.element].source_element_id:0);
}
}
Report SourceAssemblyWallCase::Impl::InitializeConnector() {
    if(!connector)return Success();
    fe::type25::BatchConfig c;c.owner=owner.accepted();
    c.configuration_id=setup.settings()->configuration_id;c.qualification_id=setup.settings()->qualification_id;
    c.element_count=bindings.connectors()->connection_count();c.max_connections=config.storage.connector.max_connections;
    c.max_nodes=config.storage.max_nodes;c.max_device_bytes=config.storage.connector.max_device_bytes;
    c.max_host_bytes=config.storage.connector.max_host_bytes;
    const auto& v=setup.settings()->initial_velocity;
    c.startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,{v[0],v[1],v[2]}};
    return ConnectorReport(connector->batch.InitializeJoined(c,*bindings.connectors(),*bindings.combined_mass()),bindings);
}
Report SourceAssemblyWallCase::Impl::AssembleConnector(const fe::NodalAssemblyView& view) {
    return connector?ConnectorReport(connector->batch.AssembleAccepted(owner,view),bindings):Success();
}
Report SourceAssemblyWallCase::Impl::ReadInitialConnector() {
    if(!connector)return Success();
    auto& values=connector->results[accepted_slot];fe::type25::BatchDiagnostics d;
    auto r=ConnectorReport(connector->batch.CopyAcceptedResults(accepted().diagnostics.stamp,values.data(),values.size(),&d),bindings);
    if(!r)return r;
    const auto& common=accepted().diagnostics.shells;
    if(!common.has_connector||!fe::type25::batch_detail::SameDiagnostics(d,common.connector))
        return Failure(Status::ComponentFailure,"Initial connector result differs from common publication");
    for(std::size_t e=0;e<values.size();++e) {
        const auto& value=values[e];const auto source=bindings.connectors()->connections()[e].source_element_id;
        for(double work:value.history.internal_work_J)if(work!=0)
            return Failure(Status::ComponentFailure,"Initial source connector has nonzero native work",source);
        if(!value.history.active||value.history.failure_criterion!=0)
            return Failure(Status::ComponentFailure,"Initial source connector is not intact",source);
        if(!std::isfinite(value.critical_dt_s)||value.critical_dt_s<=0||
           config.fixed_dt>value.critical_dt_s*config.deformation.maximum_native_dt_fraction)
            return Failure(Status::EnvelopeFailure,"Initial source connector native timestep exceeded",source,SIZE_MAX,
                           config.fixed_dt,value.critical_dt_s*config.deformation.maximum_native_dt_fraction);
    }
    return Success();
}
Report SourceAssemblyWallCase::Impl::EvaluateConnector() {
    if(!connector)return Success();
    auto& d=candidate().diagnostics.shells.connector;auto& values=connector->results[1-accepted_slot];
    auto r=ConnectorReport(connector->batch.EvaluateCandidate(owner,token,prepared,&d),bindings);if(!r)return r;
    return ConnectorReport(connector->batch.CopyPreparedResults(d,values.data(),values.size()),bindings);
}
Report SourceAssemblyWallCase::Impl::PreparePublication(fe::ShellBatchDiagnostics& common) {
    const auto& d=candidate().diagnostics.shells;
    return Convert(connector?publication.Prepare(owner,token,d.qeph,d.t3,d.connector,&common):
                             publication.Prepare(owner,token,d.qeph,d.t3,&common));
}
} // namespace crash::cases::source_assembly_dynamics
