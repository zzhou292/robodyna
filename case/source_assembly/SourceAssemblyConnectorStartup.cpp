#include "SourceAssemblyStartupInternal.h"
#include <algorithm>

namespace crash::cases::source_assembly::startup {
void Connectors(const source::SourceAssembly& source,const tl::fea::ShellBatchBinding& shells,
                const SourceAssemblyBindingOptions& options,tl::fea::type25::Model& model,
                tl::fea::NodalMassBinding& mass) {
    const auto& data=source.data();
    if(data.internal_spotwelds.empty())return;
    // The first resident TYPE25 scope supports ordinary endpoints only. Check
    // the complete authenticated membership before either native initializer;
    // TL independently authenticates actual owner membership at attachment.
    for(const auto& weld:data.internal_spotwelds)for(const auto node:weld.nodes)
        for(const auto& group:data.nodal_rigid_groups)if(group.internal&&
            std::find(group.selected_global_nodes.begin(),group.selected_global_nodes.end(),node)!=group.selected_global_nodes.end())
            throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Connectors,0,
                "Internal spotweld endpoint belongs to an unsupported active rigid group",weld.record.id);
    try {
        const source::SourceAssemblySpotweldInput adapter(source,options.source_instance_id,*options.spotweld);
        auto input=adapter.input();input.limits=options.connector_limits;
        const auto report=model.Initialize(input);
        if(!report) {
            const auto source_id=report.connection<input.connection_count?input.connections[report.connection].source_element_id:0;
            throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Connectors,unsigned(report.status),report.message,source_id);
        }
    } catch(const SourceAssemblyBindingError&) { throw; }
    catch(const std::runtime_error& error) {
        throw SourceAssemblyBindingError(SourceAssemblyBindingStage::Connectors,0,error.what());
    }
    const auto report=mass.Initialize(shells,model,options.mass_limits);
    if(!report) {
        const auto source_id=report.connection<model.connection_count()?model.connections()[report.connection].source_element_id:0;
        throw SourceAssemblyBindingError(SourceAssemblyBindingStage::CombinedMass,unsigned(report.status),report.message,source_id);
    }
}
} // namespace crash::cases::source_assembly::startup
