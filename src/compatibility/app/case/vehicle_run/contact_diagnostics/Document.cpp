#include "Document.h"
#include "output/BoundedArrayJson.h"
namespace crash::cases::vehicle_run::contact_diagnostics {
namespace {
template<std::size_t N>
void Stages(output::Document& result,const std::array<benchmarks::StageCounter,N>& counters,
            const char* const (&names)[N],bool saturated) {
    using namespace output;
    Value stages(rapidjson::kArrayType);
    for(std::size_t i=0;i<N;++i) {
        const auto& counter=counters[i];
        output::Document stage;stage.SetObject();
        String(stage,"stage",names[i]);Integer(stage,"calls",counter.calls);
        Integer(stage,"failures",counter.failures);Integer(stage,"valid_samples",counter.valid_samples);
        const bool timed=counter.calls && counter.valid_samples==counter.calls && !saturated;
        Boolean(stage,"timing_available",timed);
        if(timed) {Integer(stage,"host_wall_ns",counter.wall_ns);Integer(stage,"maximum_host_ns",counter.maximum_ns);}
        Value value;value.CopyFrom(stage,result.GetAllocator());stages.PushBack(value,result.GetAllocator());
    }
    result.AddMember("stages",stages,result.GetAllocator());
}
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
    if(phase.native_device.calls) {
        const auto& source=phase.native_device;
        output::Document device;device.SetObject();
        String(device,"scope","numerical execution; admitted includes prefetch; consumed means native staging, not physical commit");
        Integer(device,"calls",source.calls);Integer(device,"failures",source.failures);
        Integer(device,"admitted_pairs",source.admitted_pairs);Integer(device,"consumed_pairs",source.consumed_pairs);
        Integer(device,"host_pairs",source.host_pairs);Integer(device,"launches",source.launches);
        Integer(device,"scene_uploads",source.scene_uploads);Integer(device,"numeric_cohorts",source.numeric_cohorts);
        if(source.failures) {
            if(source.last_fault_cohort_begin!=SIZE_MAX)
                Integer(device,"last_fault_cohort_begin",source.last_fault_cohort_begin);
            Integer(device,"last_fault_cohort_count",source.last_fault_cohort_count);
            if(source.last_fault_pair_ordinal!=SIZE_MAX)
                Integer(device,"last_fault_pair_ordinal",source.last_fault_pair_ordinal);
        }
        array_json::Child(result,"native_device",device);
    }
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
    if(phase.discovery.timing.calls) {
        const auto& source=phase.discovery.timing;
        output::Document timing;timing.SetObject();
        Integer(timing,"calls",source.calls);
        Integer(timing,"clock_failures",source.clock_failures);
        Integer(timing,"backward_samples",source.backward_samples);
        Boolean(timing,"counter_saturated",source.counter_saturated);
        Boolean(timing,"covers_reported_calls",!phase.counter_saturated &&
            !source.counter_saturated && source.calls==phase.discovery.calls);
        Stages(timing,source.stages,DiscoveryStageNames,source.counter_saturated);
        array_json::Child(discovery,"timing",timing);
    }
    array_json::Child(result,"discovery",discovery);
    Stages(result,phase.stages,StageNames,phase.counter_saturated);
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
