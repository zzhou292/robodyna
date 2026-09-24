#include "Document.h"
#include <ostream>
namespace crash::cases::vehicle_run::contact_diagnostics {
void WriteProgress(std::ostream& stream,const Snapshot& snapshot) {
    if(!snapshot.enabled)return;
    stream<<" contact_diagnostics_scope="<<(snapshot.committed_scope?"committed":"last_attempt");
    if(snapshot.committed_scope && !snapshot.phase_matches) {
        stream<<" contact_diagnostics_available=0";return;
    }
    const Phase* phases[]{&snapshot.accepted,&snapshot.candidate};
    const char* names[]{"accepted","candidate"};
    for(unsigned side=0;side<2;++side) {
        const auto& phase=*phases[side];
        if(!phase.enabled)continue;
        stream<<" contact_"<<names[side]<<"_entered="<<phase.entered
              <<" contact_"<<names[side]<<"_finished="<<phase.finished
              <<" contact_"<<names[side]<<"_authenticated="<<phase.authenticated
              <<" contact_"<<names[side]<<"_counts_complete="<<phase.counts_complete
              <<" contact_"<<names[side]<<"_succeeded="<<phase.succeeded
              <<" contact_"<<names[side]<<"_native_batches="<<phase.native_batches
              <<" contact_"<<names[side]<<"_native_submitted_pairs="<<phase.native_submitted_pairs
              <<" contact_"<<names[side]<<"_native_work="<<phase.native_work;
        for(std::size_t i=0;i<StageCount;++i) {
            const auto& counter=phase.stages[i];
            if(!counter.calls)continue;
            const bool timed=counter.valid_samples==counter.calls && !phase.counter_saturated;
            stream<<" contact_"<<names[side]<<'_'<<StageNames[i]<<"_timing_available="<<timed;
            if(timed)stream<<" contact_"<<names[side]<<'_'<<StageNames[i]<<"_host_s="<<counter.wall_ns*1e-9;
        }
    }
}
} // namespace crash::cases::vehicle_run::contact_diagnostics
