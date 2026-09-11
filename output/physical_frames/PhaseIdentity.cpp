#include "Fields.h"
namespace crash::output::physical_frames::detail {
void CheckIdentity(const Mapping& mapping,const records::Context& context,const CaptureScope& scope) {
    const auto f=Phase(scope);
    const auto& id=context.identity();
    Require(id.owner==scope.stamp.owner_id && id.configuration==scope.diagnostics.qeph.configuration_id &&
        id.qualification==scope.diagnostics.qeph.qualification_id &&
        id.source_instance==mapping.execution().physical().domain()->source_instance_id() &&
        id.source_mapping_sha256==mapping.source_mapping().digest() &&
        Bits(context.fixed_dt())==Bits(scope.stamp.fixed_dt) && context.nodes()==mapping.physical_nodes().size() &&
        context.parents().size()==mapping.parents().size() && scope.stamp.node_count==mapping.physical_node_count(),
        "Physical capture context differs from actual owner/source");
    records::CheckStamp(context,f);
    if(scope.diagnostics.has_type45)Require(scope.type45_source_instance_id==id.source_instance,
        "Accepted joints belong to another physical source domain");
}
} // namespace crash::output::physical_frames::detail
