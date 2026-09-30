#pragma once
#include "SourceAssemblyWallSchema.h"
#include "lib_src/solvers/FENodalState.h"

namespace crash::output::assembly {
// Archive progress only. Accept* records an already written row/frame and is
// infallible; it never advances a solver or creates an accepted state.
struct WallArchiveSequence {
    WallArchiveRequest request;
    tl::fea::NodalStamp last;
    std::uint64_t last_frame=0;
    std::size_t frame_count=0,frame_capacity=0;
    bool prefix_frame=false;
    void CheckInterval(const tl::fea::NodalStamp& base,const tl::fea::NodalStamp& next) const;
    bool CheckFrame(const tl::fea::NodalStamp&) const;
    void CheckClose(const tl::fea::NodalStamp&,bool prefix) const;
    void AcceptInterval(const tl::fea::NodalStamp& s) noexcept {last=s;}
    void AcceptFrame(const tl::fea::NodalStamp& s,bool prefix) noexcept {last_frame=s.epoch;++frame_count;prefix_frame=prefix;}
};
} // namespace crash::output::assembly
