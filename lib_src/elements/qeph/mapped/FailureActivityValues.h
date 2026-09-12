// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../failure/ShellFailureValues.h"

namespace tl::fea::qeph::mapped {
// The mapped QEPH source admits NIP3 or explicit zero-point skins. Its failure
// policy and optional plastic section are the same inputs used by full Read.
TL_SHELL_SECTION_HD inline bool ValidFailureActivity(const ShellBatchFailureState& value,
    ShellFailurePolicy policy, const ShellBatchSectionState* section, double time) noexcept {
  using namespace shell_batch_plasticity_detail;
  return value.policy() == policy && ValidFailureEncoding(value) &&
      ValidFailureState(value,policy,section,time);
}
} // namespace tl::fea::qeph::mapped
