#include "ReplayClipping.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace crash::visual {
bool MakeReplayClipping(const ReplayCamera& camera,const ReplayBounds& bounds,ReplayClipping& out) noexcept {
    std::array<double,3> forward{},extent{};
    for(unsigned a=0;a<3;++a) {
        if(!std::isfinite(camera.position[a]) || !std::isfinite(camera.target[a]) ||
            !std::isfinite(bounds.low[a]) || !std::isfinite(bounds.high[a]) || bounds.low[a]>bounds.high[a]) return false;
        forward[a]=camera.target[a]-camera.position[a];
        extent[a]=bounds.high[a]-bounds.low[a];
    }
    ReplayClipping next;
    next.camera_distance_m=std::hypot(forward[0],forward[1],forward[2]);
    next.scene_diagonal_m=std::hypot(extent[0],extent[1],extent[2]);
    if(!(next.camera_distance_m>0) || !std::isfinite(next.camera_distance_m) ||
        !(next.scene_diagonal_m>0) || !std::isfinite(next.scene_diagonal_m)) return false;
    for(auto& x:forward) x/=next.camera_distance_m;
    next.minimum_depth_m=std::numeric_limits<double>::infinity();
    next.maximum_depth_m=-next.minimum_depth_m;
    for(unsigned corner=0;corner<8;++corner) {
        double depth=0;
        for(unsigned a=0;a<3;++a) depth+=((corner&(1u<<a)?bounds.high[a]:bounds.low[a])-camera.position[a])*forward[a];
        if(!std::isfinite(depth)) return false;
        next.minimum_depth_m=std::min(next.minimum_depth_m,depth);
        next.maximum_depth_m=std::max(next.maximum_depth_m,depth);
    }
    if(!(next.maximum_depth_m>0)) return false;
    // Scale-dependent positive near plane. If the complete box is in front,
    // leave at least half its nearest depth; retain a small plane otherwise.
    next.near_m=std::min(next.camera_distance_m,next.scene_diagonal_m)*1e-4;
    if(next.minimum_depth_m>0) next.near_m=std::min(next.near_m,.5*next.minimum_depth_m);
    next.far_m=1.05*std::max(next.maximum_depth_m,next.camera_distance_m)+.05*next.scene_diagonal_m;
    if(!(next.near_m>0) || !(next.far_m>next.near_m) || !std::isfinite(next.far_m)) return false;
    // VSG forms the projection in double, then uploads float matrix entries.
    // Reject unsupported scales instead of silently underflowing the near plane.
    const auto representable=[](double x) {
        return std::isfinite(x) && x>=std::numeric_limits<float>::min() && x<=std::numeric_limits<float>::max();
    };
    if(!representable(next.near_m) || !representable(next.far_m) ||
        !representable(next.near_m/(next.far_m-next.near_m)) ||
        !representable((next.far_m*next.near_m)/(next.far_m-next.near_m))) return false;
    out=next;return true;
}
} // namespace crash::visual
