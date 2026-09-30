#pragma once
#include "output/full_shell/FullShellVisualizationSchema.h"
#include "lib_src/solvers/FENodalState.h"
namespace crash::output::physical_frames::detail {
template<class D> void CheckAcceptedParticipant(const D& d,const tl::fea::NodalStamp& s,const full_shell::FrameStamp& f,
    std::uint64_t configuration,std::uint64_t qualification) {
    Require(d.valid && d.owner_id==s.owner_id && d.configuration_id==configuration &&
        d.qualification_id==qualification && d.epoch==s.epoch && Bits(d.time)==Bits(s.time) &&
        d.phase==decltype(d.phase)::Accepted && d.has_completed_interval==bool(s.epoch),
        "Accepted participant phase/owner identity differs");
    if(s.epoch) Require(d.base_epoch==f.base_epoch && d.attempt==f.attempt &&
        Bits(d.base_time)==Bits(f.base_time) && Bits(d.velocity_time)==Bits(f.velocity_time) &&
        Bits(d.kick_dt)==Bits(f.kick_dt),"Accepted participant has another interval phase");
}
}
