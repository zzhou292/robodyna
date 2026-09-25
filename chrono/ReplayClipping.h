#pragma once
#include "AcceptedReplayScene.h"
namespace crash::visual {
struct ReplayBounds {std::array<double,3> low{},high{};};
struct ReplayClipping {
    double near_m=0,far_m=0;
    double camera_distance_m=0,scene_diagonal_m=0;
    double minimum_depth_m=0,maximum_depth_m=0;
};
// Conservative depth bounds from all archived positions and the optional wall.
// Does not alter camera pose, field of view, world positions or geometry scale.
// A box crossing the eye plane is allowed; behind-camera geometry cannot be
// made visible by clipping. Framing still requires actual image inspection.
// Invalid/unrepresentable values leave the complete output unchanged.
bool MakeReplayClipping(const ReplayCamera&,const ReplayBounds&,ReplayClipping&) noexcept;
} // namespace crash::visual
