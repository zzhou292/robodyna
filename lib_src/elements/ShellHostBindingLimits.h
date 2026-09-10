// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tl::fea {
// Independent HOST startup admission. These bounds confer no resident capacity.
inline constexpr std::size_t MaxShellHostParents=1024;
inline constexpr std::size_t MaxShellHostNodes=2048;
struct ShellHostBindingLimits {
  std::size_t max_parents=MaxShellHostParents;
  std::size_t max_nodes=MaxShellHostNodes;
  std::size_t max_owned_bytes=4*1024*1024;
};

} // namespace tl::fea
