#pragma once
#include "T3Reference.h"

namespace tl::qualification::t3::detail {
// Shared startup/current numerical preflight only; never replacement native
// geometry. These functions preserve the qualified R1 arithmetic and bounds.
bool SupportedGeometry(const std::array<Vec3, 3>& x);
bool ProperFrame(const Matrix3& frame);
}  // namespace tl::qualification::t3::detail
