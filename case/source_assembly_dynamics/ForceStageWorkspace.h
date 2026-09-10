#pragma once
#include "lib_src/solvers/FENodalState.h"
#include "lib_src/solvers/NodalForceStageSnapshot.h"
#include <vector>

namespace crash::cases::source_assembly_dynamics {
// Caller-owned observation scratch, not another mechanical state. Startup
// validates the exact active extents and byte budget before constructing this.
// Disabled mode creates no dynamic payload; enabled storage never grows.
struct ForceStageWorkspace {
    ForceStageWorkspace(bool enabled,std::size_t n,std::size_t g)
        : acceleration(enabled?6*n:0),groups(enabled?g:0) {}
    std::vector<double> acceleration;
    std::vector<tl::fea::NodalRigidGroupAccelerationSnapshot> groups;
    tl::fea::NodalPreparedView prepared;
    tl::fea::NodalForceStageSnapshotBuffer buffer() noexcept {
        if(acceleration.empty())return {};
        const auto n=acceleration.size()/6;
        return {acceleration.data(),acceleration.data()+3*n,n,groups.data(),groups.size()};
    }
};
} // namespace crash::cases::source_assembly_dynamics
