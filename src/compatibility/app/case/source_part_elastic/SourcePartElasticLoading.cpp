#include "SourcePartElasticInternal.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_part_elastic {
Report SourcePartElasticCase::Impl::InitializeLoading() {
    if(config.experiment!=Experiment::ElasticPulse) {
        for(std::size_t n=0;n<NodeCount;++n) for(unsigned a=0;a<3;++a)
            accepted.velocity[3*n+a]=config.initial_velocity[a];
        return Success(); // No pulse construction or load allocation in free flight.
    }
    // Original pulse preparation, in its original arithmetic and reduction order.
    double low=accepted.position[config.spatial_axis],high=low;
    for(std::size_t n=0;n<NodeCount;++n) {
        low=std::min(low,accepted.position[3*n+config.spatial_axis]);
        high=std::max(high,accepted.position[3*n+config.spatial_axis]);
    }
    if(!std::isfinite(high-low)||high<=low) return Failure(Status::InvalidInput,"Pulse spatial axis has no source extent");
    long double weighted=0,mass=0;
    const double middle=.5*(low+high),length=high-low;
    for(std::size_t n=0;n<NodeCount;++n) {
        const double xi=2*(accepted.position[3*n+config.spatial_axis]-middle)/length;
        spatial_shape[n]=xi*xi;
        weighted+=static_cast<long double>(binding.nodes()[n].native.mass)*spatial_shape[n];
        mass+=binding.nodes()[n].native.mass;
    }
    const double mean=static_cast<double>(weighted/mass);
    for(std::size_t n=0;n<NodeCount;++n) {
        spatial_shape[n]-=mean;
        for(unsigned a=0;a<3;++a) {
            pulse_force[3*n+a]=binding.nodes()[n].native.mass*config.acceleration*spatial_shape[n]*config.direction[a];
            if(!std::isfinite(pulse_force[3*n+a])) return Failure(Status::InvalidInput,"Pulse force construction overflow");
        }
    }
    return Success();
}
} // namespace crash::cases::source_part_elastic
