#pragma once
#include "AcceptedReplayScene.h"
namespace chrono { class ChBody; }
namespace crash::visual {
std::shared_ptr<chrono::ChTriangleMeshConnected> CopyReplayGeometry(const chrono::ChTriangleMeshConnected&);
std::shared_ptr<chrono::ChVisualShapeTriangleMesh> MakeReplayShape(
    const std::shared_ptr<chrono::ChTriangleMeshConnected>&,bool moving,bool wireframe,
    bool colors=false,bool fixed_connectivity=true);
std::shared_ptr<chrono::ChBody> MakeReplayCarrier(const char*,
    const std::shared_ptr<chrono::ChVisualShapeTriangleMesh>&);
bool MakeBoundsCamera(const std::array<double,3>& low,const std::array<double,3>& high,
    std::array<double,3> direction,double distance,ReplayVertical,ReplayView,ReplayCamera&);
} // namespace crash::visual
