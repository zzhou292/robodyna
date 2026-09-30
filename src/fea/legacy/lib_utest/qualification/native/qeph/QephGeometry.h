#pragma once
#include "QephReferenceTypes.h"

namespace tl::qualification::qeph::detail {
bool ValidGeometry(const std::array<Vec3,4>& x) noexcept;
bool ProperFrame(const Matrix3& frame) noexcept;
bool Finite(const std::array<Vec3,4>& value) noexcept;
}  // namespace tl::qualification::qeph::detail
