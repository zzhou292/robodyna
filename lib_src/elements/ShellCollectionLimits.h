// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tl::fea {
// Host reference-collection bounds. Consumers adopt these deliberately; these
// constants do not raise any resident batch, nodal owner or contact capacity.
inline constexpr std::size_t MaxShellCollectionNodes=128;
inline constexpr std::size_t MaxShellCollectionParents=128; // Q4 + T3 combined.
} // namespace tl::fea
