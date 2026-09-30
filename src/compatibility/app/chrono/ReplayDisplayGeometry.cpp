#include "ReplayDisplayGeometry.h"
#include <cmath>
namespace crash::visual {
namespace {
bool Finite(const chrono::ChVector3d& p) {
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}
double RendererCoordinate(double value) {
    // Materialize the binary32 storage used by the renderer before testing
    // triangle collapse. The narrowing is part of this validation contract.
    volatile float stored = static_cast<float>(value);
    return stored;
}
} // namespace
bool CheckReplayDisplayPositions(const std::vector<chrono::ChVector3d>& positions,
        ReplayGeometryLimits limits) {
    if (!limits.valid() || positions.empty() || positions.size() > limits.vertices) return false;
    // VSG's rendering buffers are float, and its actual GetFaceNormals uses a
    // binary64 cross product and length. Reject geometry those operations cannot
    // represent, even if it was valid for a more general archive consumer.
    for (const auto& p : positions)
        if (!Finite(p) || !std::isfinite(static_cast<float>(p.x())) ||
            !std::isfinite(static_cast<float>(p.y())) || !std::isfinite(static_cast<float>(p.z()))) return false;
    return true;
}
bool CheckReplayDisplayGeometry(const std::vector<chrono::ChVector3d>& positions,
                     const std::vector<chrono::ChVector3i>& triangles,
                     ReplayGeometryLimits limits) {
    if (!CheckReplayDisplayPositions(positions, limits) || triangles.empty() ||
        triangles.size() > limits.triangles) return false;
    for (const auto& t : triangles) {
        for (int j = 0; j < 3; ++j)
            if (t[j] < 0 || static_cast<std::size_t>(t[j]) >= positions.size()) return false;
        const auto n = chrono::Vcross(positions[t[1]] - positions[t[0]], positions[t[2]] - positions[t[0]]);
        const double length2 = n.Length2();
        if (!Finite(n) || !std::isfinite(length2) || !(length2 > 0)) return false;
        chrono::ChVector3d displayed[3];
        for (int j = 0; j < 3; ++j) {
            const auto& p = positions[t[j]];
            displayed[j] = {RendererCoordinate(p.x()), RendererCoordinate(p.y()), RendererCoordinate(p.z())};
        }
        const auto displayed_normal = chrono::Vcross(displayed[1] - displayed[0], displayed[2] - displayed[0]);
        if (!(displayed_normal.Length2() > 0)) return false;
    }
    return true;
}
} // namespace crash::visual
