#pragma once
#include "AcceptedReplayScene.h"

namespace crash::visual {
// Coordinates are in the archive's SI world frame. Only presentation changes.
struct FixedCameraInput {
    std::array<double, 3> eye{}, target{};
    ReplayVertical vertical = ReplayVertical::Z;
};

// Reject an unrepresentable or degenerate look/up basis before VSG normalizes
// it. Failure leaves the complete camera unchanged.
bool MakeFixedCamera(const FixedCameraInput&, ReplayCamera&);
} // namespace crash::visual
