#include "Observe.h"
#include "lib_src/collision/SelfContactTransactionDiagnostics.h"
namespace crash::cases::vehicle_run::contact_diagnostics {
namespace {
using Native=tlfea::contact::SelfContactAttemptDiagnostics;
using Stage=tlfea::contact::SelfContactDiagnosticStage;
static_assert(StageCount==tlfea::contact::SelfContactDiagnosticStageCount);
static_assert(static_cast<unsigned>(Stage::Setup)==0 && static_cast<unsigned>(Stage::Filtering)==1 &&
    static_cast<unsigned>(Stage::Discovery)==2 && static_cast<unsigned>(Stage::EventAssembly)==3 &&
    static_cast<unsigned>(Stage::ForceAssembly)==4 && static_cast<unsigned>(Stage::Residual)==5 &&
    static_cast<unsigned>(Stage::NativeCrossing)==6 && static_cast<unsigned>(Stage::Policy)==7 &&
    static_cast<unsigned>(Stage::Finalization)==8);
Phase CopyPhase(const Native& source) noexcept {
    Phase out;
    out.enabled=source.enabled;
    if(!out.enabled)return out;
    out.entered=source.entered;out.finished=source.finished;out.succeeded=source.succeeded;
    out.authenticated=source.authenticated;
    out.owner_id=source.owner_id;out.base_epoch=source.base_epoch;out.attempt=source.attempt;
    out.counter_saturated=source.counter_saturated;out.counts_complete=source.counts_complete;
    out.clock_failures=source.clock_failures;out.backward_samples=source.backward_samples;
    for(std::size_t i=0;i<StageCount;++i) {
        const auto& counter=source.stages[i];
        out.stages[i]={counter.calls,counter.failures,counter.valid_samples,counter.wall_ns,counter.maximum_ns};
    }
    out.discovery.calls=source.discovery.calls;
    out.discovery.failures=source.discovery.failures;
    out.discovery.triangle_references=source.discovery.triangle_references;
    out.discovery.triangles=source.discovery.triangles;
    out.discovery.vertex_references=source.discovery.vertex_references;
    out.discovery.vertices=source.discovery.vertices;
    out.discovery.edge_references=source.discovery.edge_references;
    out.discovery.edges=source.discovery.edges;
    out.discovery.raw_feature_candidates=source.discovery.raw_feature_candidates;
    out.discovery.feature_candidates=source.discovery.feature_candidates;
    out.discovery.raw_intersections=source.discovery.raw_intersections;
    out.discovery.intersections=source.discovery.intersections;
    out.discovery.potential_tasks=source.discovery.potential_tasks;
    out.discovery.local_masked_tasks=source.discovery.local_masked_tasks;
    out.discovery.exact_executed_tasks=source.discovery.exact_executed_tasks;
    out.native_batches=source.native_batches;
    out.native_submitted_pairs=source.native_submitted_pairs;
    out.native_work=source.native_work;
    return out;
}
bool Matches(const Phase& phase,std::uint64_t owner,std::uint64_t base,std::uint64_t attempt) noexcept {
    return phase.enabled && phase.entered && phase.finished && phase.succeeded && phase.authenticated &&
        phase.owner_id==owner && phase.base_epoch==base && phase.attempt==attempt;
}
}
Snapshot Copy(const tlfea::contact::SelfContactTransactionDiagnostics& source) noexcept {
    Snapshot result;
    result.enabled=source.accepted.enabled || source.candidate.enabled;
    if(result.enabled) {result.accepted=CopyPhase(source.accepted);result.candidate=CopyPhase(source.candidate);}
    return result;
}
Snapshot Committed(const tlfea::contact::SelfContactTransactionDiagnostics& source,
    std::uint64_t owner,std::uint64_t base,std::uint64_t attempt) noexcept {
    auto result=Copy(source);
    if(result.enabled) {
        result.committed_scope=true;
        result.phase_matches=Matches(result.accepted,owner,base,attempt) && Matches(result.candidate,owner,base,attempt);
    }
    return result;
}
} // namespace crash::cases::vehicle_run::contact_diagnostics
