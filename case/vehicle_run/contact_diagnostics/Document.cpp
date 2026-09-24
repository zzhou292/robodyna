#include "Document.h"
#include "output/BoundedArrayJson.h"
namespace crash::cases::vehicle_run::contact_diagnostics {
namespace {
output::Document PhaseDocument(const Phase& phase) {
    using namespace output;
    output::Document result;result.SetObject();
    Boolean(result,"enabled",phase.enabled);
    if(!phase.enabled)return result;
    Boolean(result,"entered",phase.entered);Boolean(result,"finished",phase.finished);
    Boolean(result,"succeeded",phase.succeeded);Boolean(result,"authenticated",phase.authenticated);
    Integer(result,"owner_id",phase.owner_id);Integer(result,"base_epoch",phase.base_epoch);Integer(result,"attempt",phase.attempt);
    Boolean(result,"counter_saturated",phase.counter_saturated);Boolean(result,"counts_complete",phase.counts_complete);
    Integer(result,"clock_failures",phase.clock_failures);Integer(result,"backward_samples",phase.backward_samples);
    Integer(result,"native_batches",phase.native_batches);
    Integer(result,"native_submitted_pairs",phase.native_submitted_pairs);Integer(result,"native_work",phase.native_work);
    output::Document discovery;discovery.SetObject();
    Integer(discovery,"calls",phase.discovery.calls);
    Integer(discovery,"failures",phase.discovery.failures);
    Integer(discovery,"triangle_references",phase.discovery.triangle_references);
    Integer(discovery,"triangles",phase.discovery.triangles);
    Integer(discovery,"vertex_references",phase.discovery.vertex_references);
    Integer(discovery,"vertices",phase.discovery.vertices);
    Integer(discovery,"edge_references",phase.discovery.edge_references);
    Integer(discovery,"edges",phase.discovery.edges);
    Integer(discovery,"raw_feature_candidates",phase.discovery.raw_feature_candidates);
    Integer(discovery,"feature_candidates",phase.discovery.feature_candidates);
    Integer(discovery,"raw_intersections",phase.discovery.raw_intersections);
    Integer(discovery,"intersections",phase.discovery.intersections);
    Integer(discovery,"potential_tasks",phase.discovery.potential_tasks);
    Integer(discovery,"local_masked_tasks",phase.discovery.local_masked_tasks);
    Integer(discovery,"exact_executed_tasks",phase.discovery.exact_executed_tasks);
    array_json::Child(result,"discovery",discovery);
    Value stages(rapidjson::kArrayType);
    for(std::size_t i=0;i<StageCount;++i) {
        const auto& counter=phase.stages[i];
        output::Document stage;stage.SetObject();
        String(stage,"stage",StageNames[i]);Integer(stage,"calls",counter.calls);
        Integer(stage,"failures",counter.failures);Integer(stage,"valid_samples",counter.valid_samples);
        const bool timed=counter.calls && counter.valid_samples==counter.calls && !phase.counter_saturated;
        Boolean(stage,"timing_available",timed);
        if(timed) {Integer(stage,"host_wall_ns",counter.wall_ns);Integer(stage,"maximum_host_ns",counter.maximum_ns);}
        Value value;value.CopyFrom(stage,result.GetAllocator());stages.PushBack(value,result.GetAllocator());
    }
    result.AddMember("stages",stages,result.GetAllocator());
    return result;
}
}
output::Document Document(const Snapshot& snapshot) {
    output::Document result;result.SetObject();
    output::Boolean(result,"enabled",snapshot.enabled);
    if(!snapshot.enabled)return result;
    output::String(result,"scope",snapshot.committed_scope
        ? "last commonly committed interval; host elapsed scopes include existing waits; no GPU timings or physics authority"
        : "last contact attempt, including failure; candidate success is not common publication; host elapsed includes existing waits");
    output::String(result,"counts_scope","executed calls or traversed prefix; complete call reports do not establish a complete vehicle census");
    if(snapshot.committed_scope) {
        output::Boolean(result,"matches_committed_interval",snapshot.phase_matches);
        if(!snapshot.phase_matches)return result;
    }
    output::array_json::Child(result,"accepted",PhaseDocument(snapshot.accepted));
    output::array_json::Child(result,"candidate",PhaseDocument(snapshot.candidate));
    return result;
}
} // namespace crash::cases::vehicle_run::contact_diagnostics
