// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../FENodalState.h"
#include "../../math/Quaternion.h"
#include <type_traits>

namespace tl::fea::cin_advance::drift {
// Separate lifetime from recovery: every row is rewritten only after complete
// recovery succeeds. stored_axes includes the failing position axis, if any.
struct Row {
  double position[3]{};
  tl::math::Quaternion orientation;
  NodalStatus status = NodalStatus::Ok;
  std::uint32_t stored_axes = 0;
};
static_assert(sizeof(Row) == 64);
static_assert(alignof(Row) == 8);
static_assert(std::is_trivially_copyable_v<Row>);
} // namespace tl::fea::cin_advance::drift
