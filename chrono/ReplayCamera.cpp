#include "ReplayVisuals.h"
#include "ReplayFixedCamera.h"
#include <cmath>
namespace crash::visual {
bool MakeFixedCamera(const FixedCameraInput& input, ReplayCamera& camera) {
    if (input.vertical != ReplayVertical::Y && input.vertical != ReplayVertical::Z) return false;
    std::array<double, 3> forward;
    for (unsigned i = 0; i < 3; ++i) {
        if (!std::isfinite(input.eye[i]) || !std::isfinite(input.target[i])) return false;
        forward[i] = input.target[i] - input.eye[i];
        if (!std::isfinite(forward[i])) return false;
    }
    const auto length_squared = [](const auto& v) { return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]; };
    const double squared = length_squared(forward);
    if (!(squared > 0) || !std::isfinite(squared)) return false;
    const double length = std::sqrt(squared);
    for (double& value : forward) value /= length;
    // Same axis-up basis as VSG lookAt: normalize(cross(forward, up)).
    std::array<double, 3> side = input.vertical == ReplayVertical::Y
        ? std::array<double, 3>{-forward[2], 0, forward[0]}
        : std::array<double, 3>{forward[1], -forward[0], 0};
    const double side_squared = length_squared(side);
    if (!(side_squared > 0) || !std::isfinite(side_squared)) return false;
    const double side_length = std::sqrt(side_squared);
    for (double& value : side) value /= side_length;
    std::array<double, 3> up{
        side[1]*forward[2] - side[2]*forward[1],
        side[2]*forward[0] - side[0]*forward[2],
        side[0]*forward[1] - side[1]*forward[0]};
    const double up_squared = length_squared(up);
    if (!(up_squared > 0) || !std::isfinite(up_squared)) return false;
    const double up_length = std::sqrt(up_squared);
    for (double& value : up) value /= up_length;
    for (const auto& axis : {side, up, forward}) {
        const double offset = axis[0]*input.eye[0] + axis[1]*input.eye[1] + axis[2]*input.eye[2];
        if (!std::isfinite(offset)) return false;
    }
    ReplayCamera next;
    next.position = input.eye;
    next.target = input.target;
    next.vertical = input.vertical;
    next.view = ReplayView::ExplicitFixed;
    camera = next;
    return true;
}

bool MakeBoundsCamera(const std::array<double,3>& low,const std::array<double,3>& high,
        std::array<double,3> direction,double distance,ReplayVertical vertical,ReplayView view,ReplayCamera& camera) {
    ReplayCamera next;
    double extent[3];
    for (int i=0;i<3;++i) {
        const double lo=low[i],hi=high[i];
        if(!std::isfinite(lo) || !std::isfinite(hi) || hi<lo) return false;
        next.target[i]=lo*.5+hi*.5;
        extent[i]=hi-lo;
        if(!std::isfinite(extent[i])) return false;
    }
    const double diagonal=std::hypot(extent[0],extent[1],extent[2]);
    if(!(diagonal>0) || !std::isfinite(diagonal) || !(distance>0)) return false;
    if(view!=ReplayView::IncidentSide && view!=ReplayView::WallSide) return false;
    if(vertical!=ReplayVertical::Y && vertical!=ReplayVertical::Z) return false;
    next.vertical=vertical;
    next.view=view;
    if(view==ReplayView::WallSide) direction[0]=-direction[0];
    for(int i=0;i<3;++i) {
        next.position[i]=next.target[i]+distance*diagonal*direction[i];
        if(!std::isfinite(next.position[i])) return false;
    }
    camera=next;
    return true;
}
} // namespace crash::visual
