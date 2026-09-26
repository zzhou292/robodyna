#pragma once
#include "Config.h"
namespace crash::cases::vehicle_runtime::detail {
template<class C> void SetParticipantIdentity(C& out,const Config& config,
    const tl::fea::NodalStamp& stamp,const tl::fea::ShellBatchStartup& startup) {
    out.owner=stamp;out.configuration_id=config.configuration_id;
    out.qualification_id=config.qualification_id;out.startup=startup;
}
template<class C> void SetCinCounts(C& out,const tl::fea::NodalCinWitnessSource& source) {
    out.cin_attachment_count=source.range_count;out.cin_witness_count=source.witness_count;
}
inline tl::fea::NodalCinStartup MakeCinStartup(const Config& config,
    const tl::fea::NodalCinWitnessSource& source,const double* mass,const double* inertia) noexcept {
    return {source.model,mass,inertia,source.ranges,source.witnesses,source.witness_count,config.qualification_id};
}
}
