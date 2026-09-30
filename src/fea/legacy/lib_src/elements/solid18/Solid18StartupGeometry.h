// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18GaussGeometry.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline Status IntegrateGeometry(StartupGeometry& geometry) noexcept {
  const Status status = CenterGeometry(geometry);
  if (status != Status::Success) return status;
  return GaussGeometry(geometry);
}
}  // namespace tl::fea::solid18::detail
