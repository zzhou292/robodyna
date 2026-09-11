#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_startup::cin_witness_detail {
TiedCinWitnessForecast Budget(std::size_t retained,std::size_t nodes,std::size_t parents,
        std::size_t attachments,std::size_t fixed,TiedCinWitnessLimits limits) {
    using output::Require;
    const TiedCinWitnessLimits ceiling;
    Require(limits.host_bytes && limits.host_bytes<=ceiling.host_bytes && limits.witnesses &&
        limits.witnesses<=ceiling.witnesses && nodes && nodes<=524288 && parents && parents<=524288 &&
        attachments && attachments<=65536,"Invalid CIN witness count or byte limits");
    TiedCinWitnessForecast next;
    const auto add=[&](std::size_t count,std::size_t width) {
        Require(width && count<=(limits.host_bytes-next.total_host_bytes)/width,
                "Complete CIN witness preparation exceeds host byte cap");
        const auto bytes=count*width;
        next.total_host_bytes+=bytes;
        return bytes;
    };
    next.retained_source_bound=add(retained,1);
    next.fixed_bytes=add(fixed,1);
    next.incidence_scratch_bytes=add(nodes+1,sizeof(std::size_t));
    next.incidence_scratch_bytes+=add(nodes,sizeof(std::size_t));
    next.incidence_scratch_bytes+=add(parents,4*sizeof(std::uint32_t));
    next.roster_capacity_bytes=add(attachments,sizeof(cin_stage::WitnessRange));
    next.roster_capacity_bytes+=add(limits.witnesses,
        sizeof(cin_stage::ActiveWitness)+sizeof(TiedCinWitnessOrigin));
    return next;
}
void CheckActualCapacity(const TiedCinWitnessData& data,const TiedCinWitnessForecast& forecast,
        TiedCinWitnessLimits limits) {
    std::size_t bytes=0;
    const auto add=[&](std::size_t count,std::size_t width) {
        output::Require(count<=(limits.host_bytes-bytes)/width,"Actual CIN witness allocation overflows");
        bytes+=count*width;
    };
    add(data.ranges.capacity(),sizeof(cin_stage::WitnessRange));
    add(data.witnesses.capacity(),sizeof(cin_stage::ActiveWitness));
    add(data.origins.capacity(),sizeof(TiedCinWitnessOrigin));
    output::Require(bytes<=forecast.roster_capacity_bytes,"Actual CIN witness capacities exceed preflight");
}
}
