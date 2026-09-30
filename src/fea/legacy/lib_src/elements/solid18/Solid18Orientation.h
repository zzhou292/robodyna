// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18Types.h"
#include "lib_src/elements/solid_common/BrickFrame.h"

// Preserve existing entries while sharing native brick frame arithmetic.
namespace tl::fea::solid18::detail {
using solid_common::Component;
using solid_common::SetComponent;
using solid_common::Finite;
using solid_common::Positive;
using solid_common::Dot;
using solid_common::Cross;
using solid_common::Add;
using solid_common::Directions;
using solid_common::SignedCenterVolume;
using solid_common::Normalize;
using solid_common::Frame;
using solid_common::Local;
}  // namespace tl::fea::solid18::detail
