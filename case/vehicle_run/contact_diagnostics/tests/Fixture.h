#pragma once
#include "lib_src/collision/SelfContactTransactionDiagnostics.h"
namespace crash::cases::vehicle_run::contact_diagnostics::test {
// Inert host observation values, not a native mechanics/owner fixture.
inline tlfea::contact::SelfContactTransactionDiagnostics Input(
    std::uint64_t owner=7,std::uint64_t base=2,std::uint64_t attempt=9) {
    tlfea::contact::SelfContactTransactionDiagnostics result;
    for(auto* phase:{&result.accepted,&result.candidate}) {
        phase->enabled=phase->entered=phase->finished=phase->succeeded=phase->authenticated=true;
        phase->owner_id=owner;phase->base_epoch=base;phase->attempt=attempt;
        phase->discovery.calls=3;phase->discovery.triangle_references=64;phase->discovery.triangles=47;
        phase->discovery.vertex_references=192;phase->discovery.vertices=73;
        phase->discovery.edge_references=192;phase->discovery.edges=85;
        phase->discovery.raw_feature_candidates=251;phase->discovery.feature_candidates=177;
        phase->discovery.raw_intersections=19;phase->discovery.intersections=11;
        phase->discovery.potential_tasks=450;phase->discovery.local_masked_tasks=13;
        phase->discovery.exact_executed_tasks=437;
        phase->stages[2]={3,0,3,1000,600};
    }
    result.candidate.native_batches=2;result.candidate.native_submitted_pairs=37;result.candidate.native_work=90;
    result.candidate.stages[6]={2,0,2,1700,1100};
    return result;
}
} // namespace crash::cases::vehicle_run::contact_diagnostics::test
