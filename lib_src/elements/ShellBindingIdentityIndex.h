// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellHostBindingLimits.h"
#include "../../lib_utils/SourceIdentityIndex.h"
#include <algorithm>
#include <cstdint>
#include <limits>

namespace tl::fea::shell_binding_detail {
// Preserve existing element clients while sharing the phase-independent index.
template<std::size_t InlineCount>
using IdentityIndex=tl::util::SourceIdentityIndex<InlineCount>;

using ParentIdentityIndex=IdentityIndex<MaxShellHostParents>;
using NodeIdentityIndex=IdentityIndex<4*MaxShellHostParents>;
using NodeSeen=tl::util::BoundedStartupArray<bool,MaxShellHostNodes>;
// Counts have already passed the binding-only hard bounds before arithmetic.
inline constexpr std::size_t ScratchBytes(std::size_t q,std::size_t t,
    std::size_t nodes,bool legacy) noexcept {
  return ParentIdentityIndex::Bytes(legacy?0:q+t)+NodeIdentityIndex::Bytes(4*q+3*t)+
    sizeof(NodeSeen)+NodeSeen::ExtraBytes(nodes);
}
} // namespace tl::fea::shell_binding_detail
