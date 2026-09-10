// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchPublication.h"
#include "ShellBatchBinding.h"
#include "ShellCollectionLimits.h"
#include <type_traits>

namespace tl::fea::shell_publication_detail {
struct Model {
  std::size_t node_count=0;
  double mass[MaxShellCollectionNodes]{},inertia[MaxShellCollectionNodes]{};
  double physical[MaxShellCollectionNodes]{},added[MaxShellCollectionNodes]{};
};
struct Control {
  ShellPublicationStatus status=ShellPublicationStatus::Success;
  ShellBatchKinetic base,endpoint;
};
struct Storage { Model model; Control control; };
static_assert(std::is_trivially_copyable_v<Storage>,"Only native mass and scalar kinetic scratch");
static_assert(sizeof(Storage)<=8192,"One bounded mixed kinetic scratch allocation");
void LaunchMeasure(Storage*,NodalPreparedView);
} // namespace tl::fea::shell_publication_detail
