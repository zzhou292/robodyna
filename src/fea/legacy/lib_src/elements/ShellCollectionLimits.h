// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tl::fea {
// Existing default collection and resident capacities. Keep these unchanged
// when admitting larger HOST bindings through ShellHostBindingLimits.
inline constexpr std::size_t MaxShellCollectionNodes=128;
inline constexpr std::size_t MaxShellCollectionParents=128; // Q4 + T3 combined.
} // namespace tl::fea
