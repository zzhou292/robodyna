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
    const auto* beams=mapping.execution().model().structural_beams();
    Require(scope.diagnostics.has_beam18==bool(beams),
        "Accepted structural beam presence differs from the actual physical model");
    if(beams)Require(scope.beam18_source_instance_id==id.source_instance &&
        scope.beam18_source_instance_id==beams->source_instance_id() &&
        scope.beam18_parent_count==beams->parents().size(),
        "Accepted structural beams belong to another physical source or count");
}
} // namespace crash::output::physical_frames::detail
