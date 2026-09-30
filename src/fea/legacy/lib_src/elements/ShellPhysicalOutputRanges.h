// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../assembly/ShellPhysicalBinding.h"

namespace tl::fea::shell_physical_owner {
// Complete public retained payload ranges, including non-shell producers.
// Private implementation headers are not exposed as writable destinations.
bool OutputDisjoint(const ShellPhysicalBinding&,const void*,std::size_t) noexcept;
// Same complete producer traversal for consumers retaining only the immutable
// coefficient authority; includes its own shell/domain/model backing.
bool OutputDisjoint(const NodalCoefficientLedger&,const void*,std::size_t) noexcept;
} // namespace tl::fea::shell_physical_owner
