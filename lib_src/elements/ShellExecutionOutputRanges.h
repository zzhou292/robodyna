// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../assembly/ShellExecutionBinding.h"

namespace tl::fea::shell_execution_detail {
// Public immutable role and PART payloads retained by the execution handle.
// The complete coefficient ledger is checked by ShellPhysicalOutputRanges.
bool OutputDisjoint(const ShellExecutionBinding&,const void*,std::size_t) noexcept;
bool OutputDisjoint(const NodalRigidAssemblyBinding&,const void*,std::size_t) noexcept;
} // namespace tl::fea::shell_execution_detail
